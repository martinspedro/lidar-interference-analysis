
rosbag record /camera/camera_info /camera/image_raw /camera/parameter_descriptions /camera/parameter_updates /rosout /rosout_agg /diagnostics /scan /velodyne_packets /velodyne_points -e "/velodyne_nodelet_manager(.*)"
