# psdf command without tf_repeated_data warnings 

roslaunch psdf_ros test_psdf.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)

# goal command 

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
  stamp: {secs: 0, nsecs: 0}
pose:
  position:
    x: 27.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once


# Carla Command
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/carla/ego_vehicle/twist

roslaunch carla_twist_to_control carla_twist_to_control.launch role_name:=ego_vehicle


# Carla Parking lot goal 
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose: 
  position: 
    x: 285.6750793457031
    y: 214.0275115966797
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: -0.710353455330266
    w: 0.7038451310482668"

    