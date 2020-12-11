# ROS-FMI-DistributedSimulationEnvironment
ROS package that provides nodes for setting up a distributed simulation environment for Functional Mockup Units


## Install

To install the python_test package, just copy the *python_test* folder into your catkin workspace.

You might need to call

```
catkin_make
```

## Structure
* python_test
    * data_rotative_driver
        * FmuRotativDriver.fmu
        * Node_Configuration_Rotative_Driver.txt
    * data_turn_table
        * Controller.fmu
        * Electromechanic.fmu
        * Node_Configuration_Controller.txt (for distributed turntable)
        * Node_Configuration_Electromechanic.txt (for distributed turntable)
        * Node_Configuration_Turn_Table.txt (both fmus on a single node)
    * include
    * scripts (python scripts)
        * static (css and js files for flask)
        * templates (html files for flask)
        * flask_server.py
        * fmu_simulation.py (deprecated)
        * multi_fmu_simulation.py 
    * src
    * srv (Service definitions for ROS)
        * get_node_info.srv
        * set_configuration.srv
    * CMakeLists.txt
    * package.xml
    * README.md

## Run


First, open up a terminal and navigate to your project workspace (ex. catkin_workspace). There you should have access to a folder called "devel". You need to write 

```
source devel/setup.bash
```

in order to setup the terminal for the current ROS workspace. We can then start the flask server with 

```
rosrun python_test flask_server.py
```

Right now, the flask server does not handle killing via ctrl+c signals (TODO: This can be implemented!). Therefor you need to use the ctrl+z to stop the process and then use kill -9 <flask_server_process_id> to kill the process. Otherwise you can also use kill -9 %n where n is the number which is given each stopped process and is printed on the terminal after you pressed ctrl+z.

For the simulation nodes, we need to open a terminal and first set it up with the ROS Workspace by using

```
source devel/setup.bash
```

The nodes are started using

```
rosrun python_test multi_fmu_simulation.py
```

Now you should be able to see a running node on the dashboard which is accessed via you browser at

```
http://127.0.0.1:5000/
```
