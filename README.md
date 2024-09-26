# GM6020_arduino_driver
Arduino code to run DJI GM6020 Brushless motor https://www.robomaster.com/en-US/products/components/general/gm6020/info

The command values are sent via rosserial

## Library setup
```
$cd GM6020_arduino_driver/lib/
$rosrun rosserial_arduino make_libraries.py .
```
Reference: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

## How to use
Terminal 1
```
$ roscore
```

Terminal 2
```
$ rosrun rosserial_python serial_node.py _port:=/dev/gm6020 _baud:=115200
```

Terminal 3

```
# data: [controlmode, amplitude, frequency, cycles]
$ rostopic pub /gm6020_control std_msgs/Float32MultiArray "data: [4, 0.42, 4.0, 10.0]" --once
```

## Acknowledgement
The code's CAN configuration is based on
https://qiita.com/kjkmr/items/717e8cd2aa2987cdcee0
