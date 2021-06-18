In Ubuntu:
Launch the system:
```
$ roslaunch nxtbot nxt.launch
```
For Teleop control:
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
To get a smooth and continuous keyboard control, set angular speed and linear speed to 40 and 50 respectively. \
<kbd>i</kbd> / <kbd>,</kbd> for moving forward/backward.\
<kbd>j</kbd> / <kbd>l</kbd> for steering left/right. \
PORT B is drive motor, PORT C is steer motor. 
For debug messages:
```
$ rostopic echo rosout
```

\
In ROS Noetic VS 19 command prompt:
````
> cd catkin_ws
> devel/setup.bat
> conda activate py36
> rosrun nxtbot nxt_pub.py
````

OpenCV setup: \
In windows:
1. Download opencv binaries
2. Add opencv\build\x64\vc15\bin\ to PATH under System Variables. 
3. Add opencv\build\x64\vc15\lib\ to PATH under System Variables.

In Ubuntu: \
Check the word document 2 and 3.
