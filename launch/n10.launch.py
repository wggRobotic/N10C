from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='n10c',
            executable='n10c',
            parameters=[
                #{'image0': '/n10/intel/color/image_raw'},
                #{'image1': '/n10/rear/color'},
                {'image2': '/n10/cam_dif'},
                {'enable': '/eduard/enable'},
                #{'twist': '/n10/cmd_vel'},
                #{'barcode': '/n10/barcode'}
               ],
        ),
        Node(
            package= 'n10_cam_dif',
            executable= 'cam_dif',
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name= 'n10_barcode_reader',
            remappings=[
                ('/image', '/n10/intel/color'),
                ('/barcode','/n10/barcode')],
            output='screen'
        )
    ])
