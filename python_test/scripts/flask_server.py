#!/usr/bin/env python
from flask import Flask, render_template, request, copy_current_request_context, jsonify
from flask_socketio import SocketIO, send, emit
from std_msgs.msg import String
from python_test.srv import get_node_info, set_configuration
from  amlx_to_buxxe import amlx_to_buxxe

import json
import rospy
import rosservice
import os
import zipfile 


app = Flask(__name__)
app.config['SECRET_KEY'] = 'spear_secret'
socketio = SocketIO(app)

streaming_ros_subscriptions = {}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/handle_form', methods=['POST'])
def handle_form():
    print("inaide handle form")
    print("Posted file: {}".format(request.files['file']))
    fpath=os.path.dirname(os.path.abspath(__file__))
    file = request.files['file']
    fmuRefClassPath = 'AutomationMLInterfaceClassLib/AutomationMLBaseInterface/ExternalDataConnector/AutomationMLComponentInterfaceRoleClassLib/FMUVariableRef'
    files = {'file': file.read()}
    if not zipfile.is_zipfile(file):
        return jsonify("not zip file")
    else:
        f = zipfile.ZipFile(file, "r")
        encoding = 'utf-8-sig'
        #amlx_to_buxxe(file,"tmp/")
        return jsonify(amlx_to_buxxe(file,"tmp/"))

#@socketio.on("test1")
#def test1_function(file):
    #return "success from test1"

#@socketio.on('my event')
#def test_message(message):
   # socketio.emit('my response', {'data': message['data']})



@socketio.on("handle_form_test")
def handle_form_test_function(file):
    fpath=os.path.dirname(os.path.abspath(__file__))
    fmuRefClassPath = 'AutomationMLInterfaceClassLib/AutomationMLBaseInterface/ExternalDataConnector/AutomationMLComponentInterfaceRoleClassLib/FMUVariableRef'
    files = {'file': file.read()}
    if not zipfile.is_zipfile(file):
        return jsonify("not zip file")
    else:
        f = zipfile.ZipFile(file, "r")
        encoding = 'utf-8-sig'
        result=amlx_to_buxxe(file,"tmp/")
        return jsonify(result)

##########################
## Node functions START ##
##########################
# INFO: this is a potential attack vector! sanitize the id access?
# functions have a 5 sec timeout if the service is not available

@socketio.on("set_configurations")
def set_configuration_function(id, data):
    try:
        rospy.wait_for_service(id+'/set_configuration',5)
        srv=rospy.ServiceProxy(id+'/set_configuration', set_configuration)
        result = json.dumps({"msg":srv(json.dumps(data)).json})
        socketio.emit("node_configured", id)
    except Exception, e:
        result = json.dumps({"error":str(e)})
  
    return result

@socketio.on("get_node_info")
def get_node_info_function(id):
    try:
        rospy.wait_for_service(id+'/get_node_info',5)
        srv=rospy.ServiceProxy(id+'/get_node_info',get_node_info)
        result = srv().json
    except Exception, e:
        result = json.dumps({"error":str(e)})
    
    return result
    
@socketio.on("get_nodes_outputs")
def get_node_outputs_function(id):
    try:
        rospy.wait_for_service(id+'/get_node_info',5)
        srv=rospy.ServiceProxy(id+'/get_node_info',get_node_info)
        result = json.dumps(json.loads(srv().json)["ros_output"].keys())
    except Exception, e:
        result = json.dumps({"error":str(e)})
    
    return result
########################
## Node functions END ##
########################

################################
## Graph stream handler START ##
################################
def stop_stream():
    global streaming_ros_subscriptions
    
    # If the client has a streamer -> unregister
    if request.sid in streaming_ros_subscriptions and streaming_ros_subscriptions[request.sid] != None:
        streaming_ros_subscriptions[request.sid].unregister()
        del streaming_ros_subscriptions[request.sid]


@socketio.on("request_graph_stream")
def request_graph_streamer(topic):
    stop_stream()
    
    # allow this function to be called out of the active flask request context
    @copy_current_request_context
    def callback(data):
        parsed = json.loads(data.data)
        payload = {"time":parsed["time"],"value": parsed["value"]}
        socketio.emit("graph_data", payload, room=request.sid);
    
    # setup subscription to FMU Output (topic)
    global streaming_ros_subscriptions
    streaming_ros_subscriptions[request.sid] = rospy.Subscriber(topic, String, callback)
    
    print "Graph stream for: ",request.sid, topic
    return True

@socketio.on("stop_graph_stream")
def stop_graph_streamer():
    stop_stream()
    print "Stop stream for: ",request.sid
    return True
##############################
## Graph stream handler END ##
##############################

##############################
## Connection handler START ##
##############################
# dashboard connect
@socketio.on("connect")
def connect_event():
    print "Connect: ",request.sid

# dashboard disconnect
@socketio.on("disconnect")
def disconnect_event():
    print "Disconnect: ",request.sid
    stop_stream()
    
# if a dashboard connects, the server sends out the list of available simulation nodes
@socketio.on("dashboard_connect")
def dashboard_connect():
    service_list = rosservice.get_service_list(include_nodes=True)
    simulation_nodes = filter(lambda entry: "fmu_simulation" in entry[0] and "set_configuration" in entry[0],service_list)
    # simulation nodes list is: [ ["service_name",[node_name]],... ]
    return [e[1][0] for e in simulation_nodes]
############################
## Connection handler END ##
############################

#####################################
## ROS Subscription handlers START ##
#####################################
# publish the new node id to the clients
def new_node_connected(message):
    print "new node connected", message
    node_info = json.loads(message.data)
    socketio.emit("new_node_connected", node_info["node_id"])
    
# publish the disconnected node id to the clients
def node_disconnected(message):
    print "node disconnected", message
    node_info = json.loads(message.data)
    socketio.emit("node_disconnected", node_info["node_id"])

###################################
## ROS Subscription handlers END ##
###################################


if __name__ == '__main__':
    rospy.init_node('flask_dashboard', anonymous=True)

    rospy.Subscriber('/dashboard_new_node', String, new_node_connected)
    rospy.Subscriber('/dashboard_node_disconnect', String, node_disconnected)
    
    socketio.run(app, use_reloader=True)
