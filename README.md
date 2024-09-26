# GM6020_arduino_driver
Arduino code to run DJI GM6020 Brushless motor https://www.robomaster.com/en-US/products/components/general/gm6020/info

The command values are sent via rosserial

## Library setup
```
$cd GM6020_arduino_driver/lib/
$rosrun rosserial_arduino make_libraries.py .
```
Reference: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

## Acknowledgement
The code's CAN configuration is based on
https://qiita.com/kjkmr/items/717e8cd2aa2987cdcee0
