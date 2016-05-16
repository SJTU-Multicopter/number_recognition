# teleop_twist_keyboard
Generic Keyboard Teleop for ROS
#Launch
To run: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

#Usage
```
AR Drone control by keyboard
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

For yaw control, hold down the shift key:
---------------------------
   J    K    L

; : up (+z)
. : down (-z)

T : takeoff
L : landing
P : emergency stop/reset

anything else : stop

1/2 : increase/decrease max speeds by 10%
3/4 : increase/decrease only linear speed by 10%
5/6 : increase/decrease only angular speed by 10%

CTRL-C to quit
```

