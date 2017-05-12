# ECE 573 - Beta Release
Tianyang Chen, Yawei Ding, Yuanzhengyu Li

## 1 Compile and run the project

Open a terminal, type the following command line:

	$ git clone https://github.com/TianyangChen/DDCAR.git
	$ cd ddcar/BetaRelease/src/
	$ git clone https://github.com/sprinkjm/catvehicle.git
	$ git clone https://github.com/sprinkjm/obstaclestopper.git
	$ catkin_init_workspace
	$ cd ..
	$ catkin_make
	$ source devel/setup.bash
	$ roslaunch cvchallenge_task3 catvehicle_custom.launch worldfile:=world1.world
	$ gzclient 
	$ roslaunch cvchallenge_task3 task3.launch

To view the generated world file, shut down all launch file, copy the file `~/.ros/gazebo_output.world` to `/BetaRelease/src/cvchallenge_task3/worlds` and type the following command line:

	$ roslaunch cvchallenge_task3 catvehicle_custom.launch worldfile:=gazebo_output.world
	$ gzclient
	
## 2 Tests

We have 5 requirements:

1. The vehicle can stop at last.
2. The vehicle won't crash during the movement.
3. The vehicle can tell the number of barrier.
4. The vehicle can tell the position of barrier.
5. The vehicle can tell us what the barrier is.

We wrote 5 tests: 

1. `all_tests.py` is for requirement 1
2. `test_safty.py` is for requirement 2
3. `barrier_cnt.py` is for requriement 3

To run all the tests, type the following command line:

	$ rostest cvchallenge_task3 mytest.test

## 3 Youtube Link

Due to the time limits of the video, we omit the process of `git clone` and `catkin_make`, which is the same as in Alpha Release video. The video start with command 

	$ roslaunch cvchallenge_task3 catvehicle_custom.launch worldfile:=world1.world 
	
and only run one of these tests.

https://youtu.be/e8ouPxQk0zs