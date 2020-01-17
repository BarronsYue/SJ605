SJ605 Package Tutorial
========
## Setting up the environment
### Environment
* Ubuntu 16.04 LTS OS
* ROS Kinetic 1.12.14
* Gazebo 7.16.0 (least 7.7.0)
* RVIZ version 1.12.17 (Qt version 5.5.1 & OGRE version 1.9.0)

### Check environment installation
1. Check the version of gazebo
```
$ gazebo --version
``` 
2. To run projects you need version 7.7.0+. If your gazebo version is not 7.7.0+,please update your gazebo
```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```
3. Check the version again
```
$ gazebo --version
``` 
## Package installation
### Installation steps
1. Create ROS workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
2. Clone 
```
git clone url
```
3. Install ROS dependencies using the rosdep install command
```
$ cd ~/catkin_ws/
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
4. Run catkin_make from within your workspace to build the project
```
$ cd ~/catkin_ws
$ catkin_make
```
### simulation

* To run simulator by using Rviz
```
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch sj605_description sj605_rviz.launch
```
You can play around with different joint angles by adjusting the gui bars.
  
![](https://i.imgur.com/OjT5MjD.png)
  
To terminate the program, please press ctrl-c, which is also applied to all the following simulation methods.
  
* To run simulator by using Gazebo

```
cd catkin_ws
source devel/setup.bash
```
Run the code above every time to source your setup.*sh file if you open a new terminal.
  
Then run the code
```
$ roslaunch sj605_gazebo sj605_world.launch
```
You will see sj605 robot in a simulation world.
   
![](https://i.imgur.com/HADlUXY.png)

Open a new terminal, run the code to control joints of the robot.
```
$ roslaunch sj605_control sj605_control.launch
```
Now, you can try to control your robot with python or C++ by publishing some information to it.
Open a new terminal, then

* To run demo code with python
```
$ rosrun sj605_control control.py 
```
The message shown in terminal will guide you to give proper input angles.
  
![](https://i.imgur.com/l4vycSG.png)
  
* To run demo code with c++
```
$ rosrun sj605_control sj605_instruction 
```
Read the message shown in the terminal to give proper input instructions.
In this program, you can specify the position of end point of sj605. 
You are supposed to give three coordinates of the position as well as three rotation angles in the order of rotating along x, y, and then z axis.

 
Forward kinematics: 
  ![](https://i.imgur.com/MYhx1EN.png)
  
Inverse kinematics:
![](https://i.imgur.com/I94pAcj.png)
 
If your output message of inverse kinematics contain 'nan' or 'Exceed joint limit', that means your input position and orientation is not in the reachable region of the robot,  you should just try to type another valid input.


## Robot information
### DH table of SJ605

| Links | i   | alpha(i-1) | a(i-1) | d(i)   | theta(i) |
| ----- | --- |:----------:|:------:|:------:|:--------:|
| 0->1  | 1   |     0      |   0    |  0.9   |    q1    |
| 1->2  | 2   |     90     |   0    | 1.081  |  90+q2   |
| 2->3  | 3   |     0      |  3.75  | -1     |    q3    |
| 3->4  | 4   |     90     |   0    | 0.27   |    q4    |
| 4->5  | 5   |    -90     |   0    | 0      |    q5    |
| 5->6  | 6   |     90     |   0    | 0.131  |    q6    |
| 6->7  | 7   |     0      |   0    | 0      |    0     |

