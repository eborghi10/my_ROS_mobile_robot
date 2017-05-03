# robot_msgs::Motor

```
bool name
float32 data
```

## How to use custom messages in the Arduino IDE

In order to know if the package was compiled successfuly:

`~$ rosmsg list | grep robot_msgs`

*NOTE:* _This is my personal implementation and is the only method that worked._

`~$ rosrun rosserial_arduino make_library.py folder_name`

Then browse into _folder_name_: `/folder_name/ros_lib/`

Finally, copy your `robot_msgs` folder into your `/libraries/ros_lib/` folder corresponding to the Arduino IDE.