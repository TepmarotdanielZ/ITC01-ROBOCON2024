
from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
   
        # Node(package='abu_can', executable= 'can2'),
        
        # Node(package='abu_can', executable= 'can'),
        Node(package='farmer_robot', executable='ps4'),
    
        Node(package='farmer_robot', executable='can'),
        
        Node(package='farmer_robot', executable='robot_blue'),
        
        # Node(package='rqt_image_view', executable='rqt_image_view'),
    ])