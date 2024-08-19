
from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
   
        # Node(package='abu_can', executable= 'can2'),
        
        # Node(package='abu_can', executable= 'can'),
        Node(package='farmer_robot', executable='ps4', output = 'log'),
    
        Node(package='farmer_robot', executable='can', output= 'log'),
        
        Node(package='farmer_robot', executable='robot_red'),
        
        # Node(package='rqt_image_view', executable='rqt_image_view'),
    ])