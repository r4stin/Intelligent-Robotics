# Intelligent-Robotics @UNIPD

To run 1st Assignment:

- open five terminals and go to catkin_ws folder (">cd catkin_ws")
- run ">source devel/setup.bash" in each terminal.
- on terminal one run ">roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library"
- on terminal two run ">roslaunch assignment2 all.launch"
- wait for a bit to setup everything...
- on terminal three run ">roslaunch assignment2 assignment2.launch"
- on terminal four run ">rosrun assignment2 node_B"
- on terminal five run ">rosrun assignment2 node_C"

*Before running 2nd assignment, go to "assignment1/src/action_server.cpp". Comment first part of the code , then uncomment second part of code which is separated from fisrt part by "ASSIGNMENT 2".*

To run 2nd Assignment:


1) roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables

2) roslaunch assignment2 all.launch

3) roslaunch assignment2 assignment2.launch

4) rosrun assignment2 node_B

5) rosrun assignment2 node_C

