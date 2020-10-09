## How to setup OpenCPN for visualisation:
#### (These instructions assume that you've already got local_pathfinding set up withing the `src` directory of a catkin workspace. Also, the `catkin_make` and `source` commands have been omitted.)

1. **Install OpenCPN**
```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository ppa:opencpn/opencpn
$ sudo apt update
$ sudo apt install opencpn
```

2. **Configure OpenCPN to listen to UDP port `localhost:65500`**

Press the Options button
![DeepinSkjermbilde_velg-område_20200125191044](https://user-images.githubusercontent.com/1413378/73130065-721f5b00-3fa6-11ea-9b8f-b517839fd508.png)

Under connections, add a new UDP connection at `0.0.0.0:65500`. Make sure to check _Receive Input on this Port_. 

![DeepinSkjermbilde_Navigator_20200125185322](https://user-images.githubusercontent.com/1413378/73130059-4c925180-3fa6-11ea-8641-68c91c29135c.png)

Click _OK_ and close the _Options_ window. You can (but shouldn't have to) confirm that OpenCPN is listening by using `netstat` from the `net-tools` package:
```bash
$ sudo netstat -lp | grep opencpn
udp        0      0 0.0.0.0:65500           0.0.0.0:*                           14613/opencpn      
```

3. **Run the mock ROS nodes**
Run the following commands in (preferably in separate terminals, or use `screen` or `tmux`)

```bash
$ roscore
$ rosrun local_pathfinding MOCK_AIS.py
$ rosrun local_pathfinding MOCK_controller_and_boat.py
$ rosrun local_pathfinding MOCK_UDP_bridge.py
```

4. **Launch OpenCPN**
The boats should now be visible in the OpenCPN map. The yellow arrows are AIS units.
![DeepinSkjermbilde_Navigator_20200126225929](https://user-images.githubusercontent.com/1413378/73155457-973bc880-408f-11ea-8228-d449d85b00dd.png)


5. **BONUS: Configure OpenCPN to show wind direction and speed**
Open the _Options_ window again, open the _Plugins_ tab. Enable the _Dashboard_ plugin, and hit _Preferences_.
![DeepinSkjermbilde_urxvt_20200126213457](https://user-images.githubusercontent.com/1413378/73152252-0d3a3280-4084-11ea-961e-2a810e224d14.png)

Enable _Show this dashboard_, and add the _True Wind Angle and Speed_ instrument. Remove the others. Hit _OK_, and finally hit _OK_ again to close the _Options_ window.
![DeepinSkjermbilde_urxvt_20200126213601](https://user-images.githubusercontent.com/1413378/73152281-25aa4d00-4084-11ea-8f63-709ff6dd278d.png)
