## MRL-AMRL Barrel Detector

this package detect barrel/s with 2D lidar data.

### usage

On this assumption you have an Ubuntu 16.04(Xenial) and you have installed ROS Kinetic Kame. You need a package for detecting barrels.

### Download the pakage
```markdown
cd ~/wokspace/src
git clone https://github.com/SaraTohidi/barrel_detector.git
```
### Build the workspace
```markdown
cd ..
catkin_make
```
Now you may need to get some dependencies.

### Get some dependencies
```markdown
ros-kinetic-nav-msgs
ros-kinetic-dynamic-reconfigure
ros-kinetic-OpenCV
```
