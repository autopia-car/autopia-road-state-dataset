from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    camera_front_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.161445", "0.0174674", "-0.257808", "0.018211", "0.145767", "-0.00378974", "rubyplus", "camera_front"])

    camera_front_left_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.171841", "0.142724", "-0.26274", "0.693539", "0.120547", "0.0217674", "rubyplus", "camera_left"])

    camera_front_right_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0.142016", "-0.131321", "-0.241215", "-0.551337", "0.169142", "-0.0122094", "rubyplus", "camera_right"])

    rubyplus_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["1.21", "0.0", "0.31", "0.0", "0.0", "0.0", "gps", "rubyplus"])

    helios_left_tf = Node(package = "tf2_ros", 
                   executable = "static_transform_publisher",
                   arguments = ["-0.390", "0.550", "-0.200", "1.644", "0.148", "0.006", "rubyplus", "helios_left"])

    helios_right_tf = Node(package = "tf2_ros", 
               executable = "static_transform_publisher",
               arguments = ["-0.350", "-0.550", "-0.220", "-1.513", "0.140", "0.009", "rubyplus", "helios_right"])

    imu_left_tf = Node(package = "tf2_ros", 
                   executable = "static_transform_publisher",
                   arguments = ["2.50", "-0.75", "-1.10", "0.0", "0.0", "0.0", "gps", "imu_left"])

    imu_right_tf = Node(package = "tf2_ros", 
               executable = "static_transform_publisher",
               arguments = ["2.50", "0.75", "-1.10", "0.0", "0.0", "0.0", "gps", "imu_right"])

    imu_center_tf = Node(package = "tf2_ros", 
               executable = "static_transform_publisher",
               arguments = ["3.30", "0.0", "-1.33", "0.0", "0.0", "0.0", "gps", "imu_center"])

    gps_tf = Node(package = "tf2_ros", 
               executable = "static_transform_publisher",
               arguments = ["0.15", "0.0", "1.25", "0.0", "0.0", "0.0", "odom", "gps"])

    ld.add_action(rubyplus_tf)
    ld.add_action(helios_left_tf)
    ld.add_action(helios_right_tf)
    ld.add_action(camera_front_tf)
    ld.add_action(camera_front_left_tf)
    ld.add_action(camera_front_right_tf)
    ld.add_action(imu_left_tf)
    ld.add_action(imu_right_tf)
    ld.add_action(imu_center_tf)
    ld.add_action(gps_tf)

    return ld