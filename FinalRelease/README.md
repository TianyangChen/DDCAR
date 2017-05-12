# ECE 573 - Final Release
Tianyang Chen, Yawei Ding, Yuanzhengyu Li

## 1 Compile and run the project

Open a terminal, type the following command line:

	$ git clone https://github.com/TianyangChen/DDCAR.git
	$ cd ddcar/FinalRelease/src/
	$ git clone https://github.com/sprinkjm/catvehicle.git
	$ git clone https://github.com/sprinkjm/obstaclestopper.git
	$ catkin_init_workspace
	$ cd ..
	$ catkin_make
	$ source devel/setup.bash
	$ roslaunch ddcar catvehicle_custom.launch worldfile:=world6.world
	$ gzclient 
	$ roslaunch ddcar ddcar.launch

To view the generated world file, shut down all launch file, and type the following command line:

	$ roscore
	$ rosrun gazebo_ros gazebo ~/.ros/gazebo_output.world
	
## 2 Tests

We have 5 requirements:

1. B requirement: DDCAR can avoid all the barriers automatically while running.
2. B requirement: DDCAR can automatically stop in the end.
3. B requirement: DDCAR can detect the number of the objects in the visual field of the vehicle as expected.
4. A requirement: DDCAR can recognize the type of the objects. These types in our project are House, Jersey barrier and Construction cone.
5. A requirement: DDCAR can detect the position of the objects in its visual field as the real world.

We wrote 5 tests: 

1. `test_safty.py` is for requirement 1
2. `test_vehicle_stop.py` is for requirement 2
3. `barrier_cnt.py` is for requriement 3
4. `test_object_type.py` is for requirement 4
5. `test_object_pose.py` is for requirement 5

To run all the tests, type the following command line:

	$ rostest ddcar run_all_tests.test

Run all tests may take a long time, you can also run them separately:

	$ rostest ddcar test_safty.test
	$ rostest ddcar test_vehicle_stop.test
	$ rostest ddcar test_barrier_cnt.test
	$ rostest ddcar test_object_type.test
	$ rostest ddcar test_object_pose.test

The file `run_all_tests.test` is in directory `/FinalRelease/src/ddcar/test`.

In the file `run_all_tests.test`, we use `world6.world` as the default simulated world file. You can modify this in `run_all_tests.test`.

Once you modified the default world file, don't foget to modify parameters `expected_barrier_num` and `expected_objects` in `run_all_tests.test`.

For more details, please read the comment in `run_all_tests.test`.

## 3 Youtube Link

https://youtu.be/_ioVstFZPhk

