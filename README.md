# Event-based-tactile-sensor

First of all you need to download metavision SDK libraries following the instructions from the [Prophesee official page](https://www.prophesee.ai/metavision-intelligence-essentials-download-2-1/)


# Requirements

  * Ubuntu 20.04 or 18.04
  * ROS Noetic or ROS Melodic
  * Metavision Essentials 2.1


  * Clone the source code to your catkin workspace

    ```
        cd catkin_ws/src
        git clone https://github.com/nurlando04/Event-based-tactile-sensor
        cd ..
    ```

  * Compile

    ```
        catkin_make
    ```

  * Source the workspace

    ```
        source ~/catkin_ws/devel/setup.bash
    ```
 

## Getting Started
You simply need to rosrun the C++ file
```
rosrun tracking prophesee_ros_spatter
```
If you want to tune the camera you can add that option and add path to the bias file

```
rosrun tracking prophesee_ros_spatter -b ~/catkin_ws/src/tracking/out.bias 

```

Instructions on how to create the bias can be found [here](https://docs.prophesee.ai/2.1.0/metavision_player/biases.html)

