FROM ros:foxy

# copy selected packages to desired directory in container
COPY . /root/ament_ws/src/rviz2_rotatable_image_plugin

SHELL ["bash", "-c"]

# install dependencies
RUN apt update && \
  apt install python3-colcon-common-extensions -y && \
  cd /root/ament_ws && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y

# build repo
RUN . /ros_entrypoint.sh && cd /root/ament_ws && \
  colcon build && \
  sed -i '$isource "/root/ament_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
