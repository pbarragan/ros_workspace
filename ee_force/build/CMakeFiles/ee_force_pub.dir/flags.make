# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# compile CXX with /usr/bin/c++
CXX_FLAGS = -O2 -g -I/home/barragan/pr2_repo/ros_workspace/ee_force/include -I/home/barragan/ros_packages/mit-ros-pkg/branches/pr2_demos/pr2_utils/msg_gen/cpp/include -I/opt/ros/fuerte/include -I/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/include -I/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/geometry/tf/include -I/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/bullet/include -I/opt/ros/fuerte/stacks/geometry/angles/include -I/home/barragan/pr2_repo/ros_workspace/orocos_kinematics_dynamics/orocos_kdl/install_dir/include -I/usr/include/eigen3 -I/opt/ros/fuerte/stacks/pr2_mechanism/pr2_controller_manager/include -I/opt/ros/fuerte/stacks/pr2_mechanism/pr2_hardware_interface/include -I/opt/ros/fuerte/stacks/pr2_mechanism/pr2_mechanism_model/include -I/opt/ros/fuerte/stacks/robot_model/urdf/include -I/opt/ros/fuerte/stacks/robot_model/colladadom/include -I/opt/ros/fuerte/stacks/robot_model/colladadom/include/1.5 -I/opt/ros/fuerte/stacks/robot_model/urdf_parser/include -I/opt/ros/fuerte/stacks/robot_model/urdf_interface/include -I/opt/ros/fuerte/stacks/robot_model/collada_parser/include -I/opt/ros/fuerte/stacks/robot_model/kdl_parser/include -I/opt/ros/fuerte/stacks/pluginlib/include -I/opt/ros/fuerte/stacks/pluginlib -I/opt/ros/fuerte/stacks/ros_control/hardware_interface/include -I/opt/ros/fuerte/stacks/diagnostics/diagnostic_updater/include -I/opt/ros/fuerte/stacks/pr2_mechanism/pr2_mechanism_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_mechanism/pr2_mechanism_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_mechanism/pr2_controller_interface/include -I/opt/ros/fuerte/stacks/ros_control/controller_interface/include -I/opt/ros/fuerte/stacks/robot_model/robot_state_publisher/include -I/opt/ros/fuerte/stacks/geometry/tf_conversions/include -I/opt/ros/fuerte/stacks/ros_control/realtime_tools/include -I/opt/ros/fuerte/share/actionlib/include -I/home/barragan/ros_packages/mit-ros-pkg/stacks/fuerte/trunk/ee_cart_imped/ee_cart_imped_msgs/msg_gen/cpp/include -I/home/barragan/ros_packages/mit-ros-pkg/stacks/fuerte/trunk/ee_cart_imped/ee_cart_imped_action/include -I/opt/ros/fuerte/stacks/pr2_common/pr2_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_common/pr2_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation/kinematics_msgs/include -I/opt/ros/fuerte/stacks/arm_navigation/kinematics_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation/kinematics_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_controllers/pr2_controllers_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/object_manipulation/object_manipulation_msgs/include -I/opt/ros/fuerte/stacks/object_manipulation/object_manipulation_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/object_manipulation/object_manipulation_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/object_manipulation/object_manipulator/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/interpolated_ik_motion_planner/srv/cpp -I/opt/ros/fuerte/stacks/arm_navigation_experimental/interpolated_ik_motion_planner/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/geometry/eigen_conversions/include -I/opt/ros/fuerte/stacks/arm_navigation/planning_environment/include -I/opt/ros/fuerte/stacks/arm_navigation/geometric_shapes/include -I/opt/ros/fuerte/stacks/robot_model/resource_retriever/include -I/opt/ros/fuerte/stacks/arm_navigation/planning_models/include -I/opt/ros/fuerte/stacks/arm_navigation/collision_space/include -I/opt/ros/fuerte/stacks/physics_ode/opende/opende/include -I/opt/ros/fuerte/stacks/physics_ode/opende/threadpool -I/opt/ros/fuerte/include/pcl-1.5 -I/opt/ros/fuerte/stacks/perception_pcl/pcl_ros/include -I/opt/ros/fuerte/stacks/perception_pcl/pcl_ros/cfg/cpp -I/opt/ros/fuerte/stacks/nodelet_core/nodelet/include -I/opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/bond_core/bondcpp/include -I/opt/ros/fuerte/stacks/bond_core/bond/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/bond_core/smclib/include -I/opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation/robot_self_filter/include -I/opt/ros/fuerte/stacks/filters/include -I/opt/ros/fuerte/stacks/visualization/rviz/src -I/opt/ros/fuerte/stacks/python_qt_binding/include -I/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/include -I/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/include/OGRE -I/opt/ros/fuerte/stacks/visualization_common/ogre/ogre/include/OGRE/RTShaderSystem -I/opt/ros/fuerte/stacks/visualization_common/ogre_tools/src -I/opt/ros/fuerte/stacks/laser_pipeline/laser_geometry/include -I/opt/ros/fuerte/stacks/image_common/image_transport/include -I/opt/ros/fuerte/stacks/visualization/interactive_markers/include -I/opt/ros/fuerte/stacks/arm_navigation/arm_kinematics_constraint_aware/include -I/opt/ros/fuerte/stacks/arm_navigation/kinematics_base/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/msg_gen/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_collision_map_processing/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_collision_map_processing/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include -I/opt/ros/fuerte/stacks/sql_database/database_interface/include -I/opt/ros/fuerte/stacks/pr2_controllers/pr2_gripper_action/msg/cpp -I/opt/ros/fuerte/stacks/pr2_controllers/pr2_mechanism_controllers/include -I/opt/ros/fuerte/stacks/pr2_controllers/pr2_mechanism_controllers/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_controllers/pr2_mechanism_controllers/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_controllers/robot_mechanism_controllers/include -I/opt/ros/fuerte/stacks/pr2_controllers/robot_mechanism_controllers/msg/cpp -I/opt/ros/fuerte/stacks/pr2_controllers/robot_mechanism_controllers/srv/cpp -I/opt/ros/fuerte/stacks/pr2_controllers/robot_mechanism_controllers/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/ros_control/control_toolbox/include -I/opt/ros/fuerte/stacks/ros_control/control_toolbox/include/control_toolbox/eigen2 -I/opt/ros/fuerte/stacks/ros_control/control_toolbox/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/control/control_msgs/include -I/opt/ros/fuerte/stacks/control/control_msgs/msg/cpp -I/opt/ros/fuerte/stacks/control/control_msgs/srv/cpp -I/opt/ros/fuerte/stacks/control/control_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_manipulation_controllers/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_manipulation_controllers/srv/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_manipulation_controllers/msg/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_manipulation_controllers/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_manipulation_controllers/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/ros_realtime/rosrt/include -I/opt/ros/fuerte/stacks/ros_realtime/rosatomic/include -I/opt/ros/fuerte/stacks/ros_realtime/allocators/include -I/opt/ros/fuerte/stacks/ros_realtime/lockfree/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_wrappers/include -I/opt/ros/fuerte/stacks/navigation/move_base_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_common_actions/pr2_common_action_msgs/msg/cpp -I/opt/ros/fuerte/stacks/pr2_common_actions/pr2_common_action_msgs/srv/cpp -I/opt/ros/fuerte/stacks/pr2_common_actions/pr2_common_action_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_controllers/joint_trajectory_action/msg/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_action/msg/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_controller/msg/cpp -I/opt/ros/fuerte/stacks/pr2_arm_navigation/pr2_arm_navigation_perception/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/laser_pipeline/laser_filters/include -I/opt/ros/fuerte/stacks/laser_pipeline/laser_assembler/include -I/opt/ros/fuerte/stacks/laser_pipeline/laser_assembler/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/collider/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/collider/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/octomap_ros/include -I/opt/ros/fuerte/include/opencv -I/opt/ros/fuerte/stacks/vision_opencv/image_geometry/include -I/opt/ros/fuerte/stacks/image_pipeline/stereo_image_proc/include -I/opt/ros/fuerte/stacks/image_pipeline/stereo_image_proc/cfg/cpp -I/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include -I/opt/ros/fuerte/stacks/image_pipeline/image_proc/include -I/opt/ros/fuerte/stacks/image_pipeline/image_proc/cfg/cpp -I/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/chomp_motion_planner/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/chomp_motion_planner/srv/cpp -I/opt/ros/fuerte/stacks/arm_navigation_experimental/chomp_motion_planner/msg/cpp -I/opt/ros/fuerte/stacks/arm_navigation_experimental/chomp_motion_planner/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/chomp_motion_planner/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation/spline_smoother/include -I/opt/ros/fuerte/stacks/arm_navigation/spline_smoother/msg/cpp -I/opt/ros/fuerte/stacks/arm_navigation/spline_smoother/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/collision_proximity/include -I/opt/ros/fuerte/stacks/arm_navigation/constraint_aware_spline_smoother/include -I/opt/ros/fuerte/stacks/arm_navigation/ompl_ros_interface/msg/cpp -I/opt/ros/fuerte/stacks/arm_navigation/ompl_ros_interface/include -I/opt/ros/fuerte/stacks/arm_navigation/ompl_ros_interface/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation/trajectory_filter_server/include -I/opt/ros/fuerte/stacks/arm_navigation/move_arm/msg/cpp -I/opt/ros/fuerte/stacks/arm_navigation/move_arm/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/move_arm_warehouse/msg/cpp -I/opt/ros/fuerte/stacks/warehousewg/mongo_ros/include -I/opt/ros/fuerte/stacks/warehousewg/mongodb/mongo/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/move_arm_head_monitor/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/arm_navigation_experimental/head_monitor_msgs/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/pr2_kinematics/pr2_arm_kinematics_constraint_aware/include -I/opt/ros/fuerte/stacks/pr2_kinematics/pr2_arm_kinematics/include -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_grasp_planner_cluster/srv/cpp -I/opt/ros/fuerte/stacks/pr2_object_manipulation/manipulation/pr2_gripper_grasp_planner_cluster/srv_gen/cpp/include -I/home/barragan/pr2_repo/ros_workspace/ee_force/msg_gen/cpp/include    -DROS_PACKAGE_NAME='"ee_force"'

CXX_DEFINES = 

# TARGET_FLAGS = -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -DdDOUBLE -pthread -DOCTOMAP_NODEBUGOUT -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread
