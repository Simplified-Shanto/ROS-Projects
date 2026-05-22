#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from gazebo_msgs.srv import SpawnEntity, SetEntityState
import os, math 
from ament_index_python.packages import get_package_share_directory
from ruamel.yaml import YAML
from geometry_msgs.msg import Pose, Quaternion, Twist

class TrackMaker(Node): # TrackMaker inherits from node, that gives access to things like create_subscription(), create_publisher(), createa_client(), get_logger())
    def __init__(self): 
        super().__init__('track_maker')
        package_directory = get_package_share_directory('wall_bot_gazebo')
        default_tower_template_path = os.path.join(package_directory, 'config', 'object_template.sdf')
        default_settings_path = os.path.join(package_directory, 'config', 'track.yaml')

        self.declare_parameter('tower_template_path', default_tower_template_path)
        self.declare_parameter('settings_path', default_settings_path)

        self.tower_template_path = self.get_parameter('tower_template_path').get_parameter_value().string_value
        self.settings_path = self.get_parameter('settings_path').get_parameter_value().string_value

        self.tower_template = ""
        self.settings = {}
        self.count = 1 #Tower counter used for spawning towers 

        yaml = YAML()

        #### Load the YAML settings file 
        if os.path.isfile(self.settings_path):
            with open(self.settings_path, 'r') as f: 
                self.settings = yaml.load(f)
        else: 
            self.get_logger().error(f"Settings file '{self.settings_path}' does not exist.")
            return 

        if not isinstance(self.settings, dict): 
            self.get_logger().error(f"Settings file '{self.settings_path}' did not load into a dictionary.")
            return 

        self.get_logger().info("Waiting for Services to be available...")
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.spawn_client.wait_for_service()
        self.get_logger().info("SpawnEntity service is available.")
       
        #### Load the tower template path 
        if os.path.isfile(self.tower_template_path): 
            with open(self.tower_template_path, 'r') as f: 
                self.tower_template = f.read()
                self.get_logger().info(f"Template loaded from {self.tower_template_path}")
        else: 
            self.get_logger().warn(f"Template file '{self.tower_template_path}' does not exist.")

        if self.settings.get('objects') and not self.tower_template: 
            self.get_logger().error("Tower spawning is enabled, but the tower template could not be loaded.")
            return
        
        #### Spawning the towers 
        if self.settings.get('objects') == True: 
            for tower in self.settings['towers']: 
                self.get_logger().info(f"Spawning tower: {tower}")
                self.spawn_tower(tower)
        

    def spawn_tower(self, tower): 
        position = (
            float(tower['pos']['x']), 
            float(tower['pos']['y']), 
            0.5, 
        )

        sdf = self.tower_template.format(
            color=tower['color']
        )

        req = SpawnEntity.Request()
        req.name = f'tower_{self.count}'
        req.xml = sdf 
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose = Pose()
        req.initial_pose.position.x = position[0]
        req.initial_pose.position.y = position[1]
        req.initial_pose.position.z = position[2]

        resp = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

        if resp.result().success: 
            self.get_logger().info(f'Tower {self.count} spawned successfully!')
            self.count +=1
        else: 
            self.get_logger().error(f'Failed to spawn tower: {resp.result().status_message}')

def main(): 
    rclpy.init()
    node = TrackMaker()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__': 
    main()
