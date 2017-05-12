task2 node: 
 1)could detect the obstacles publish the range of the obstacle to car through sensor laser 
 2)it could how many obstacles in the world but there still some problems of coordinate 
easyrun node: 
 1)could get the data from the task2 node
 2) according our own algorithm to avoid the obstacles,by changing linear velocity and angular velocity.The car will never get collision. 
 3)the user to initializes the velocity parameters of the car

The list of requirements has finished : 1,5,4

$ git clone https://github.com/TianyangChen/DDCAR.git
$ cd ddcar/AlphaRelease/src/
$ git clone https://github.com/sprinkjm/catvehicle.git
$ git clone https://github.com/sprinkjm/obstaclestopper.git
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ roslaunch cvchallenge_task2 catvehicle_custom.launch worldfile:=world1.world
$ rosrun rviz rviz 
$ roslaunch cvchallenge_task2 alpha.launch

Youtube Link:
https://youtu.be/Q-TBkbTOtbM
