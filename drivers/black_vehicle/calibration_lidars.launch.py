"""Self-contained black-vehicle lidar driver and initial sensor TF tree."""
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    configs = root / 'config/black_vehicle'
    sensor = yaml.safe_load((configs / 'sensor_kit_calibration.yaml').read_text())['sensor_kit_base_link']
    base = yaml.safe_load((configs / 'sensors_calibration.yaml').read_text())['base_link']['sensor_kit_base_link']
    robot = Element('robot', name='black_vehicle_calibration_sensors')
    SubElement(robot, 'link', name='base_link')
    def joint(parent, child, pose):
        SubElement(robot, 'link', name=child)
        j = SubElement(robot, 'joint', name=child+'_mount', type='fixed')
        SubElement(j, 'parent', link=parent)
        SubElement(j, 'child', link=child)
        SubElement(j, 'origin', xyz=' '.join(str(pose[k]) for k in ('x','y','z')),
                   rpy=' '.join(str(pose[k]) for k in ('roll','pitch','yaw')))
    joint('base_link', 'sensor_kit_base_link', base)
    for name in ('top','left','right','rear'):
        child = f'velodyne_{name}_base_link'
        joint('sensor_kit_base_link', child, sensor[child])
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='black_vehicle_calibration_tf', parameters=[{'robot_description': tostring(robot, encoding='unicode')}]),
        Node(package='rslidar_sdk', executable='rslidar_sdk_node', name='black_vehicle_lidars',
             output='screen', additional_env={'RSLIDAR_CONFIG_PATH':str(configs/'rslidar.yaml')})
    ])
