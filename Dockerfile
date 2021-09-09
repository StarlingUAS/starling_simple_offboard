ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

COPY simple_offboard /ros_ws/src/
COPY simple_offboard_msgs /ros_ws/src/

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select simple_offboard_msgs simple_offboard \
    && rm -r build

CMD ["ros2", "launch", "simple_offboard", "simple_offboard.launch.xml"]