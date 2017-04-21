# robot_msgs::Arduino

```
string name
float32 data
```

## How to use custom messages in the Arduino IDE

*NOTE:* _This is my personal implementation and was the only method that worked._

`~$ rosrun rosserial_arduino make_library.py folder_name`

Then browse into _folder_name_: `/folder_name/ros_lib/`

Finally, copy your _robot_msgs_ folder into your _/libraries/ros_lib/_ folder corresponding to the Arduino IDE.