#!/usr/bin/python3
#-*- coding: utf-8 -*-

import rospy, roslaunch, numpy as np

class Spawn:
    def __init__(self):
        self.PACKAGE_NAME = 'simulation_vsss'
        self.SPAWN_LAUNCHFILE = 'spawn_robot.launch'

        self.COLORS = ["blue", "yellow"]

        self.FORMATION_3X3 = {
            self.COLORS[0]: [(-0.2, 0), (-0.5, 0.3), (-0.5, -0.3)],
            self.COLORS[1]: [(0.2, 0), (0.5, 0.3), (0.5, -0.3)]
        }
        
        # Config roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Get launchfiles
        node_name = rospy.get_name()

        robots_per_team = rospy.get_param(f"{node_name}/robots_per_team")
        team_color = rospy.get_param(f"{node_name}/team_color")

        roslaunch_files = self.get_launchfiles(node_name, robots_per_team, team_color)

        for file in roslaunch_files:
            launch = roslaunch.parent.ROSLaunchParent(uuid, [file])
            launch.start()

    def get_yaw(self, color):
        if color == self.COLORS[0]: return 0
        else: return np.pi
    
    def get_is_yellow(self, color):
        if color == "yellow": return "true"
        else: return "false"
    
    def get_launchfiles(self, node_name, robots_per_team, team_color):
        roslaunch_files = []
        formations = self.FORMATION_3X3

        for i in range(robots_per_team):
            cli_args = [
                self.PACKAGE_NAME,
                self.SPAWN_LAUNCHFILE,
                f'robot_number:={i}',
                f'is_yellow:={self.get_is_yellow(team_color)}',
                f'robot_name:={team_color}/{i}',
                f'x:={formations[team_color][i][0]}',
                f'y:={formations[team_color][i][1]}',
                f'yaw:={self.get_yaw(team_color)}',
                f'model:={rospy.get_param(f"{node_name}/model")}',
                f'controller_config_file:={rospy.get_param(f"{node_name}/controller_config_file")}',
                f'ros_control_config_file:={rospy.get_param(f"{node_name}/ros_control_config_file")}',
                f'namespace:={team_color}/{i}'
            ]

            roslaunch_args = cli_args[2:]
            roslaunch_file = (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)

            roslaunch_files.append(roslaunch_file)

        return roslaunch_files
    
def main():
    rospy.init_node('robots_spawner', anonymous=True)
    node = Spawn()
    rospy.spin()

if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass