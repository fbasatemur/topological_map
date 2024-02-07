# topological_map

## Project Description

Voronoi graph extraction for the TurtleBot3 house environment has been implemented using the tuw_voronoi_graph package in ROS 1 Noetic system.

## Environment Setup

The packages slam, navigation, and description should be utilized from the [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3.git) address.

The following adjustments should be made for these packages in the following order:

```bash
mkdir -p ~/tmap/src
cd ~/tmap/src
git init
git remote add -f origin https://github.com/ROBOTIS-GIT/turtlebot3.git
git config core.sparseCheckout true
echo "turtlebot3_description" >> .git/info/sparse-checkout
echo "turtlebot3_navigation" >> .git/info/sparse-checkout
echo "turtlebot3_slam" >> .git/info/sparse-checkout
git pull origin master
rm -r .git
```

To create the topological map, the tuw_multi_robot/tuw_voronoi_graph package will be used. Requirements for the tuw_multi_robot package:

```bash
sudo apt install libdxflib-dev
export ROS_VERSION=kinetic  # for Ubuntu 16.04
export ROS_VERSION=melodic  # for Ubuntu 18.04
export ROS_VERSION=noetic   # for Ubuntu 20.04
```

```bash
sudo apt install ros-$ROS_VERSION-map-server
sudo apt install ros-$ROS_VERSION-stage-ros
```

```bash
export MRRP_DIR=$HOME/projects/catkin/tuw_multi_robot/
mkdir -p $MRRP_DIR/src
cd $MRRP_DIR/src
git clone --branch $ROS_VERSION git@github.com:tuw-robotics/tuw_multi_robot.git 
git clone --branch $ROS_VERSION git@github.com:tuw-robotics/tuw_geometry.git 
git clone git@github.com:tuw-robotics/tuw_msgs.git 
```

Modification of the description and navigation packages:

```bash
sudo nano turtlebot3_description/urdf/turtlebot3_waffle.urdf.xacro
```

```html
<joint name="scan_joint" type="fixed">
  <parent link="base_link"/>
  <child link="base_scan"/>
  <origin xyz="-0.064 0 0.2" rpy="0 0 0"/>    <!-- only this line-->
</joint>
```

```bash
sudo nano turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro
```

```html
<gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor type="ray" name="lds_lfcd_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>$(arg laser_visual)</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120</min>
          <max>15</max>	    <!-- only this line-->
          <resolution>0.015</resolution>
        </range>
```

```bash
nano turtlebot3_navigation/launch/move_base.launch
```

Ensure that DWA planners are installed as given below

```html
  <!-- move_base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/dwa_local_planner_params_$(arg model).yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
    <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)" />
  </node>
```

Starting the simulation environments

```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation move_base.launch
```

Installation of the topological_map repository and launching of the node:

```bash
cd ~/tmap/src
git clone https://github.com/fbasatemur/topological_map.git
mv topological_map-main/tuw_voronoi_graph .
rm -r topological_map-main
cd ..
catkin_make
source devel/setup.bash
rosrun tuw_voronoi_graph tuw_voronoi_graph_node
# or ./devel/lib/tuw_voronoi_graph/tuw_voronoi_graph_node
```

## Creating the topological map
The Voronoi map calculated using the tuw_voronoi_graph package is published in the sensor_msgs format via the publishVoronoiMap method on the voronoi_mapping_node topic.

```cpp
void publishVoronoiMap(cv::Mat& voronoi_map){
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", voronoi_map).toImageMsg();
    voronoi_map_pub.publish(*msg);
}
```

https://github.com/fbasatemur/topological_map/blob/main/doc/

## Result for turtlebot3_house

<p float="left">
  <img src="https://github.com/fbasatemur/topological_map/blob/main/doc/result_voronoi_map.jpg?ref_type=heads" width="400" height="400"/>
  <img src="https://github.com/fbasatemur/topological_map/blob/main/doc/turtlebot_3_voronio_map.png?ref_type=heads" width="400" height="400"/> 
</p>

![t3_voronoi_map_ss](https://github.com/fbasatemur/topological_map/blob/main/doc/voronoi_mapping_finally.png?ref_type=heads)









