# rviz2_rotatable_image_plugin

[![](https://github.com/LKSeng/rviz2_rotatable_image_plugin/workflows/rviz2_rotatable_image_plugin_ci/badge.svg?branch=main)](https://github.com/LKSeng/rviz2_rotatable_image_plugin/actions)

RViz plugin to rotate `sensor_msgs::msg::Image` for visualisation purposes in-situ, especially so if camera is mounted at an angle. For avoidance of doubt, note that this plugin does not publish the rotated image.

Alternative topologies include launching an [image_rotate node](https://wiki.ros.org/image_rotate) to rotate and publish the rotated image, and then subscribing said rotated image topic on RViz with the regular image display plugin.

This plugin works by taking the `sensor_msgs::msg::Image` before it is rendered, converting it to `cv::Mat` if rotation is needed for OpenCV to perform the rotation, and converting the rotated image back to `sensor_msgs::msg::Image` for rendering.

This plugin was created for [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html), on [Ubuntu-20.04](https://releases.ubuntu.com/20.04/). For the ROS version of this node, check out [rviz_rotatable_image_plugin](https://github.com/LKSeng/rviz_rotatable_image_plugin).

# Installation

1. Clone this repository to your ROS workspace
2. Build this package
    ```shell
    colcon build --packages-select rviz2_rotatable_image_plugin
    ```

# Usage

Select this plugin in RViz.

![rviz_rotatable_image_plugin in RViz](images/select_plugin.png "Plugin appears as ImageRotated under namespace rviz_rotatable_image_plugin in RViz")

# Examples

| Original | 45° Rotation | 45° Rotation No Crop |
| ----------- | ----------- | ----------- |
| ![Plugin with no rotation](images/original.png "No rotation") | ![Plugin with rotation](images/rotate_crop.png "Rotated image") | ![Plugin with rotation without crop](images/rotate_no_crop.png "Rotated image without crop option") |

<!-- 
# Debugging

Check if this plugin is installed properly:

```shell
rospack plugins --attrib=plugin rviz
```

You should see something like:
```shell
rviz_rotatable_image_plugin /home/user/catkin_ws/src/rviz_rotatable_image_plugin/plugin_description.xml
rviz /opt/ros/noetic/share/rviz/plugin_description.xml
```
-->

# Acknowledgments

This plugin would not have been possible if not for the contributions from the following (and contributions therein):

1. [lucasw's answer regarding wrapping Qt headers, with code](https://answers.ros.org/question/206363/rqt-plugin-undefined-symbol/?answer=232435#post-id-232435)
2. [Rotating image without crop in C++ OpenCV](https://stackoverflow.com/a/24352524)
3. The people that contributed to OpenCV and `rviz_default_plugins::displays::Image`
4. Mah weekend
