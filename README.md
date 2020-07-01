# ros_deep_grasp

This is the ROS implementation of our RA-L work '[Real-world Multi-object, Multi-grasp Detection](https://github.com/ivalab/grasp_multiObject_multiGrasp)'. The detector takes RGB-D image input and predicts multiple grasp candidates for a single object or multiple objects, in a single shot. The original arxiv paper can be found [here](https://arxiv.org/pdf/1802.00520.pdf). The final version will be updated after publication process.

If you find it helpful for your research, please consider citing:

    @inproceedings{chu2018deep,
      title = {Real-World Multiobject, Multigrasp Detection},
      author = {F. Chu and R. Xu and P. A. Vela},
      journal = {IEEE Robotics and Automation Letters},
      year = {2018},
      volume = {3},
      number = {4},
      pages = {3355-3362},
      DOI = {10.1109/LRA.2018.2852777},
      ISSN = {2377-3766},
      month = {Oct}
    }
    
If you encounter any questions, please contact me at fujenchu[at]gatech[dot]edu

### Installation 
 - Please follow the instructions in our [original repo](https://github.com/ivalab/grasp_multiObject_multiGrasp) for DeepGrasp
 - Please follow the instructions in [ROS website](https://www.ros.org/) to install ROS (we tested on Indigo)
 - Please follow the instructions in [wiki](http://wiki.ros.org/freenect_launch) to install freenect

### Usage
open ROS master
```
roscore
```
run freenect for vision input
```
roslaunch freenect_launch freenect.launch
```
run grasp node
```
rosrun ros_deep_grasp grasp.py --net res50 --dataset grasp
```

### Acknowledgement
Thanks to [Anina Mu](https://github.com/aninamu) and all ORS18-19 team members ([Joshua](https://github.com/josterdude), [Nicholas](https://github.com/nflint7) and [Jianni](https://github.com/jadkisson6)) to develop this wrapper.
