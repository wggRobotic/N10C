import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='n10c',
            executable='n10c',
            name='n10c',
            parameters=[{
                'image0': '/n10/front/color',
                'image1': '/n10/rear/color',
                'image2': '/n10/cam_dif',
                'image3': '/n10/depth/color',
                'image4': '/n10/thermal/color',
                'image5': '/n10/intel/color/image_raw',
                'twist': '/n10/cmd_vel',
                'gripper': '/n10/arm_state',
                # 'barcode': '/n10/barcode',
                'enable': '/eduard/enable',
            }]
        ),
        Node(
            package='n10_cam_dif',
            executable='cam_dif',
            name='cam_dif'
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='n10_barcode_reader',
            remappings=[
                ('/image', '/n10/intel/color/image_raw'),
                ('/barcode', '/n10/barcode')
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
