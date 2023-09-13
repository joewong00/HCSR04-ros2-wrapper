# HCSR04 Ultrasonic Sensor ROS2 Driver

## Parameters
| Name            | Type  | Description                      | Default value  |
| :-------------- | :---- | :------------------------------- | :------------- |
| minimum_range   | int   |  Detection minimum range         | 2              |
| maximum_range   | int   |  Detection maximum range         | 100            | 
| field_of_view   | float |  Detection Field of View         | 15.0           |
| trigger_pin     | int   |  Wiring Pi GPIO pin for trigger  | 25             |
| echo_pin        | int   |  Wiring Pi GPIO pin for echo     | 24             |

## Subscriptions/Publications
```
Node [/sonar_driver_node]
Publications:
* /ultrasonic/distance [sensor_msgs/Range]
* /ultrasonic/relative_velocity [sensor_msgs/Range]

Subscriptions:
```

## To Run
### Install Wiring Pi
Follow this site: https://www.electronicwings.com/raspberry-pi/how-to-use-wiringpi-library-on-raspberry-pi

### Install Packages to Workspace
```
cd ~/ros2_ws/src/
git clone https://github.com/joewong00/HCSR04-ros2-wrapper.git
cd ..
colcon build
source install/setup.bash
```

### Launch
```
ros2 launch ultrasonic_driver ultrasonic.launch.py
```