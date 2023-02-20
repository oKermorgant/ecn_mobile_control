from simple_launch import SimpleLauncher
from launch.substitutions import Command
from os import system

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('robot', default_value='r2d2', description='r2d2 / bike / two_steering')
    
    sl.include('map_simulator', 'simulation2d_launch.py',
               launch_arguments={'map': sl.find('mobro', 'void.yaml'),
                                 'display': True})
               
    sl.node('rviz2', arguments=['-d', sl.find('mobro', 'config.rviz')])
    
    sl.node('rqt_reconfigure')
               
    with sl.group(ns='robot'):
    
        sl.robot_state_publisher('mobro', sl.name_join(sl.arg('robot'), '.urdf'), 'urdf')
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True, 'force_laser': False})
        sl.node('map_simulator', 'kinematics.py')
       
    return sl.launch_description()
