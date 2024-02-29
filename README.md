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
