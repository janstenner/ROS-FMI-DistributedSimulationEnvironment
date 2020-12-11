#!/usr/bin/env python
from std_msgs.msg import String
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
from fmpy.util import plot_result
import numpy as np
import time as module_time
import rospy
import shutil
from Queue import Queue
import threading
import simplejson
import os
import socket
from python_test.srv import get_node_info, get_node_infoResponse
from python_test.srv import set_configuration, set_configurationResponse  
import base64 
  
class fmu:
    def __init__(self, filename, instance_name, input_connectors, output_connectors, start_time):
        self.filename = filename
        self.instance_name = instance_name

        self.unzipdir = extract(filename)
        self.model_description = read_model_description(filename, validate=False)
        
        self.variables = {}
        for variable in self.model_description.modelVariables:
            self.variables[variable.name] = variable
            
        # create connector dicts
        self.in_connectors = {connector_name : {"value": self.variables[connector_name].start} for connector_name in input_connectors}
        self.out_connectors = {connector_name : {"value": 0.0} for connector_name in output_connectors}
        
        fmu_kwargs = {'guid': self.model_description.guid,
                  'unzipDirectory': self.unzipdir,
                  'modelIdentifier': self.model_description.coSimulation.modelIdentifier,
                  'instanceName': instance_name}
                  
        self.fmu = FMU2Slave(**fmu_kwargs)
        self.fmu.instantiate()
        self.fmu.setupExperiment(startTime=start_time)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()
        
    def do_step(self, time, step_size):
        # relay the connector values into the fmu
        for connector_name, entry in self.in_connectors.items():
            self.set_value(connector_name, entry["value"])
            
        # do step
        self.fmu.doStep(currentCommunicationPoint=time, communicationStepSize=step_size)

        # get outputs
        for connector_name, entry in self.out_connectors.items():
            entry["value"] = self.get_value(connector_name)
    
    def set_value(self, name, value):
        vr = [self.variables[name].valueReference]
        if self.variables[name].type == 'Real':
            self.fmu.setReal(vr, [float(value)])
        elif self.variables[name].type in ['Integer', 'Enumeration']:
            self.fmu.setInteger(vr, [int(value)])
        elif self.variables[name].type == 'Boolean':
            if isinstance(value, basestring) and value.lower() == 'false':
                self.fmu.setBoolean(vr, [False])
            else:
                self.fmu.setBoolean(vr, [value != 0.0])
                
    def get_value(self, name):
        vr = [self.variables[name].valueReference]
        if self.variables[name].type == 'Real':
            return self.fmu.getReal(vr)[0]
        elif self.variables[name].type in ['Integer', 'Enumeration']:
            return self.fmu.getInteger(vr)[0]
        elif self.variables[name].type == 'Boolean':
            value = self.fmu.getBoolean(vr)[0]
            return value != 0
    
    def clean_up(self):
        self.fmu.terminate()
        self.fmu.freeInstance()
        shutil.rmtree(self.unzipdir)

  
class multi_fmu_simulation:
    def __init__(self, fmus_config,connections_config, ros_input_route, input_types, ros_output_routes={}):
        self.timequeue = Queue()
        self.lasttime = 0.0
        
        self.lock = threading.Lock()
        
        self.input_types = input_types
        self.input_last_value_times = {}
        self.valuequeues = {}
        self.nextelements = {}
        for inputname in self.input_types:
            self.input_last_value_times[inputname] = 0.0
            self.valuequeues[inputname] = Queue()
            self.nextelements[inputname] = {}
        
        # publishers for each ros path 
        self.ros_output_routes = ros_output_routes
        self.publishers = {}

        for ros_route in ros_output_routes:
            # TODO: make type dynamic?
            self.publishers[ros_route] = rospy.Publisher(ros_route, String, queue_size=20)
        
        self.time = module_time.time() # if we start at 0.0 that might make problems?
        
        # we setup all fmus
        self.fmus={}
        for fmu_filename, fmu_instance, inputs, outputs in fmus_config:
            self.fmus[fmu_instance] = fmu("tmp/"+fmu_filename,fmu_instance,inputs,outputs,self.time)
        
        self.connectors = {}
        for current_fmu_name, cfmu  in self.fmus.items():
            self.connectors[current_fmu_name] = cfmu.in_connectors.copy()
            self.connectors[current_fmu_name].update(cfmu.out_connectors)
        
        # build the connections with the connector objects.
        self.connections = []
        for start,end in connections_config:
            self.connections.append((self.connectors[start[0]][start[1]],self.connectors[end[0]][end[1]]))

        # TODO: how to handle multiple subscribers parallely? 
        # simulation should only do steps when all input vars are updated, but is this guaranteed to happen?
        # assumption right now: Only one subscriber which triggers a simulation step
        self.ros_input_route = ros_input_route
        self.time_subscription = rospy.Subscriber("sim_time", String, self.timecallback)
        self.input_subscription = rospy.Subscriber(ros_input_route[0], String, self.inputcallback)


    # the callback for the time subscription
    def timecallback(self, data):
        print "got time! " + data.data
        data = simplejson.loads(data.data)
        if 'time' in data.keys() and data['time'] != 0:
            with self.lock:
                self.timequeue.put(data['time'])
                print "timequeue size: " + str(self.timequeue.qsize())
                self.checkforstep()
            
    # the callback for the input subscription
    def inputcallback(self, data):
        print "got data! " + data.data
        data = simplejson.loads(data.data)
        if 'time' in data.keys() and data['time'] != 0:
            current_message_time = data['time']
        else:
            current_message_time = self.lasttime
        
        # set the connectors values with the message data
        input_fmu,fmu_connectors = self.ros_input_route[1]
        for value in data['values']:
            if value in fmu_connectors:
                if current_message_time >= self.input_last_value_times[value]:
                    with self.lock:
                        self.valuequeues[value].put({'time': current_message_time, 'value': data['values'][value]})
                        
                        self.checkforstep()
                        
    def checkforstep(self):
        #perform value propagation
        for inputname in self.input_types:
            if not bool(self.nextelements[inputname]):
                try:
                    self.nextelements[inputname] = self.valuequeues[inputname].get()
                except Queue.Empty:
                    # Handle empty queue here
                    pass
            
            if bool(self.nextelements[inputname]):
                if self.nextelements[inputname]['time'] <= self.input_last_value_times[inputname]:
                    self.nextelements[inputname] = {}
                elif self.nextelements[inputname]['time'] <= self.lasttime:
                    self.connectors[input_fmu][inputname]["value"] = self.nextelements[inputname]['value']
                    self.input_last_value_times[inputname] = self.nextelements[inputname]['time']
        
        didstep = False
        
        if self.lasttime == 0.0:
            if not self.timequeue.empty():
                currenttime = self.timequeue.get()
                for current_fmu_name,cfmu in self.fmus.items():
                    cfmu.do_step(currenttime, 0.1)
                didstep = True
                    
        elif not self.timequeue.empty():
            #check for internal input timestamps
            internal_inputs_ready = True
            for input in self.input_types:
                if self.input_types[input] == 1:
                    if self.input_last_value_times[input] < self.lasttime:
                        internal_inputs_ready = False
            
            if internal_inputs_ready:
                currenttime = self.timequeue.get()
                step_size = currenttime - self.lasttime
                for current_fmu_name,cfmu in self.fmus.items():
                    cfmu.do_step(currenttime, step_size)
                didstep = True
        
        if didstep:
            # update connections
            for start_connector, end_connector in self.connections:
                end_connector["value"] = start_connector["value"]
            
            for ros_route,output_streams in self.ros_output_routes.items():
                for fmu_name, connector_list in output_streams:
                    out_values = {}
                    for valuename in connector_list:
                        out_values[valuename] = self.connectors[fmu_name][valuename]["value"]
                    self.publishers[ros_route].publish(simplejson.dumps({'time': currenttime, 'values': out_values}, ignore_nan=True))
            
            self.lasttime = currenttime
            self.checkforstep()
        
    def destroy(self):
        # TODO: what about publishers? can they / must they be explicitly destroyed?
        self.input_subscription.unregister()
        
        for current_fmu_name,cfmu in self.fmus.items():
            cfmu.clean_up()
        

# The simulation manager node sets itself up and informs the 
# system that its ready and waiting to receive a simulation configuration to start    
class simulation_manager_node:
    def __init__(self):
        # WARNING: This needs to be done before the publish happens, which means that we could
        # either use a rospy.sleep or simply put it above init_node (which takes some cycles; this is how the ros examples "solve" this...)
        # This seems to be pretty bad design by ROS because if you dont sleep, there is no warning and no message sent!
        self.new_node_publisher = rospy.Publisher("dashboard_new_node", String, queue_size=10, latch=True)
        self.disconnect_node_publisher = rospy.Publisher("dashboard_node_disconnect", String, queue_size=10, latch=False)
        
        # setup the node as a manager of a simulation:
        rospy.init_node('fmu_simulation', anonymous=True) # anonymous lets ROS name the node uniquely
        rospy.on_shutdown(self.shutdown_cleanup)
        
        # init with empty config
        self.init_config()
        self.running_simulation = None
        
        # register the services
        # the ~ prepends the node name as private namespace to the service name
        rospy.Service('~get_node_info',get_node_info,self.node_info)
        rospy.Service('~set_configuration',set_configuration,self.set_node_configuration)
        
        # this sleep shall ensure that the publishers are set up
        rospy.sleep(7);
        
        print "publishing..."

        # inform dashboard that node is available
        self.new_node_publisher.publish(simplejson.dumps({'node_id' : rospy.get_name(), 'hostname' : socket.gethostname()}, ignore_nan=True))
                
        # INFO: It would be nice to simply unregister the publisher to have the node clean but it seems that unregistering
        # a publisher results in more problems. This issue is adressed here: https://github.com/ros/ros_comm/issues/111
        # Still no fix in sight and therefor we simply leave the publisher dangling
        # new_node_publisher.unregister()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    # return node info as json string
    def node_info(self, request):
        return simplejson.dumps({ 
                            "node_id" : rospy.get_name(), 
                            "hostname" : socket.gethostname(),
                            "fmus":self.config_fmus,
                            "connections": self.config_connections,
                            "ros_input": self.config_ros_input,
                            "ros_output": self.config_ros_output
                        }, ignore_nan=True)
        
    def init_config(self):
        self.config_fmus = []
        self.config_connections = []
        self.config_ros_input = ()
        self.config_ros_output = {}
        
    # set configuration has request and response as String json
    def set_node_configuration(self, request):
        print "got config! "
        self.init_config()

        message=simplejson.loads(request.json)

        # check for running configurations and stop them
        if self.running_simulation is not None:
            self.running_simulation.destroy()
            
        # store the new fmus
        if not os.path.exists("tmp"):
            os.makedirs("tmp")
        
        # TODO: we might not guarantee that each fmu has a unique filename. 
        # maybe we need to replace the incoming filenames with unique identifiers
        # for each entry in the fmus dictionary
        for fmu_entry in message["ros_fmu_files"]:
            new_fmu = open("tmp/"+fmu_entry[0], "wb")
            data = base64.decodestring(fmu_entry[1])
            new_fmu.write(data)
            new_fmu.close()
              
        # set new config
        self.config_fmus = message["ros_fmus"]
        self.config_connections = message["ros_connections"]
        self.config_ros_input = message["ros_in"]
        self.config_ros_input_types = message["ros_in_types"]
        self.config_ros_output = message["ros_out"]
        
        # start new config
        self.running_simulation = multi_fmu_simulation(self.config_fmus, self.config_connections, self.config_ros_input, self.config_ros_input_types, self.config_ros_output)
    
        return set_configurationResponse("Configuration loaded successfully")
        
    def shutdown_cleanup(self):
        if self.running_simulation is not None:
            self.running_simulation.destroy()
            
        self.disconnect_node_publisher.publish(simplejson.dumps({'node_id' : rospy.get_name(), 'hostname' : socket.gethostname()}, ignore_nan=True))
        
        # INFO: we could also cleanup the "tmp" folder for the fmus but maybe other simulation instances use the folder, therefor we let it persist
    
        
if __name__ == '__main__':
    simulation_manager_node()
