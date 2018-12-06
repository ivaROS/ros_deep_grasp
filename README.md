roscore

roslaunch freenect_launch freenect.launch

rosrun ros_deep_grasp grasp.py --net res50 --dataset grasp
