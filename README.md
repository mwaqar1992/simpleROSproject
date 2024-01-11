# simpleROSproject

Creating a simple ROS and Gazebo project for a robot to go from one position to another involves several steps. In this example, I'll guide you through creating a basic differential drive robot that moves between two predefined positions in Gazebo. We'll use the ROS navigation stack for path planning.

1. Set Up a ROS Workspace:

mkdir -p ~/ros_gazebo_project/src
cd ~/ros_gazebo_project/src
catkin_init_workspace
cd ~/ros_gazebo_project
catkin_make
source devel/setup.bash

2. Create a Package:

cd ~/ros_gazebo_project/src
catkin_create_pkg simple_robot_gazebo roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf

3. Download a Simple Robot Model:

For simplicity, you can use an existing robot model. Download a differential drive robot model, for example, the "turtlebot3" model:

cd ~/ros_gazebo_project/src/simple_robot_gazebo
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git


4. Create Launch and Config Files:

Create a launch file for launching the Gazebo simulation:

bash
Copy code
cd ~/ros_gazebo_project/src/simple_robot_gazebo
mkdir launch
touch launch/simple_gazebo.launch
Edit simple_gazebo.launch with the following content:

xml
Copy code
<launch>
  <!-- Launch Gazebo with the turtlebot3 world -->
  <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_empty_world.launch"/>
  
  <!-- Load the navigation stack -->
  <arg name="map_file" default="$(find simple_robot_gazebo)/maps/simple_map.yaml"/>
  <include file="$(find turtlebot3_navigation)/launch/turtlebot3_navigation.launch">
    <arg name="map_file" value="$(arg map_file)"/>
  </include>

  <!-- Spawn the robot in the world -->
  <node name="spawn_robot" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -param robot_description -model turtlebot3 -x 2 -y 2 -z 0"/>
</launch>
5. Create a Simple Map:

Create a simple map for the robot to navigate. Create a new folder called maps inside simple_robot_gazebo and create a file named simple_map.yaml inside it. You can use any map editor to create a map.

6. Create a Simple Navigation Node:

Create a simple navigation node to send a goal to the robot. Create a new file named simple_navigation.cpp inside src:

cpp
Copy code
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation");

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 4.0; // Set your goal position
  goal.target_pose.pose.position.y = 4.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot reached the goal");
  } else {
    ROS_INFO("The robot failed to reach the goal");
  }

  return 0;
}

7. Modify CMakeLists.txt and package.xml:
Add the following lines to CMakeLists.txt:
add_executable(simple_navigation src/simple_navigation.cpp)
target_link_libraries(simple_navigation ${catkin_LIBRARIES})

Modify package.xml to include the required dependencies:
<build_depend>turtlebot3_navigation</build_depend>
<run_depend>turtlebot3_navigation</run_depend>

8. Build the Workspace:
cd ~/ros_gazebo_project
catkin_make

9. Launch the Simulation:
roslaunch simple_robot_gazebo simple_gazebo.launch

10. Run the Navigation Node:
Open a new terminal and run:
rosrun simple_robot_gazebo simple_navigation
The robot should move from its initial position to the specified goal position.

This is a basic example, and you can further enhance it by integrating sensors, implementing obstacle avoidance, and using more advanced navigation algorithms based on your robot's capabilities.
