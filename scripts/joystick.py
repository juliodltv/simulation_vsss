#!/usr/bin/python3
import rospy, rospkg, sys, numpy as np
sys.dont_write_bytecode = True

from std_msgs.msg import Float64, ColorRGBA
from sensor_msgs.msg import Joy

from std_srvs.srv import Empty as EmptySrv
from gazebo_msgs.srv import SetLightProperties
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from gazebo_msgs.msg import ModelStates

class Joystick:
    def __init__(self, model, topic):
        
        mapping = np.load(rospkg.RosPack().get_path('simulation_vsss')+'/scripts/modules/joysticks.npz', allow_pickle=True)
        self.buttons_keys, self.axes_keys = mapping[model].item().values()
        
        self.buttons = dict.fromkeys(self.buttons_keys, 0)
        self.axes = dict.fromkeys(self.axes_keys, 0.0)
        
        rospy.Subscriber(topic, Joy, self.joystick_callback, queue_size=1)
        
        self.observers = []
        
    def joystick_callback(self, msg:Joy):
        buttons_values = msg.buttons
        axes_values = msg.axes
        
        self.buttons = dict(zip(self.buttons_keys, buttons_values))
        self.axes = dict(zip(self.axes_keys, axes_values))
        
        self.notify_observers()
        
    def register_observer(self, observer):
        self.observers.append(observer)
        
    def notify_observers(self):
        for observer in self.observers:
            observer.update()

class JoystickVSSS:
    def __init__(self, model, topic, mode, r, b):
        self.joystick = Joystick(model=model, topic=topic)
        self.joystick.register_observer(self)
        self.light = 0.298 # Default light value
        
        self.mode = mode
        self.r, self.b = r, b
        self.vel = 30
        
        self.next_robot_button = 0
        self.prev_robot_button = 0
        self.light_up_button = 0
        self.light_down_button = 0
        
        rospy.loginfo(f'Mode: {mode}')
        
        # Teams
        yellow = []
        blue = []
        
        for i in range(3):
            yellow.append([[rospy.Publisher(f'/yellow/{i}/left_controller/command', Float64, queue_size=1)], 
                                [rospy.Publisher(f'/yellow/{i}/right_controller/command', Float64, queue_size=1)]])
            blue.append([[rospy.Publisher(f'/blue/{i}/left_controller/command', Float64, queue_size=1)], 
                              [rospy.Publisher(f'/blue/{i}/right_controller/command', Float64, queue_size=1)]])
        
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        
        self.robots = {'yellow': yellow, 'blue': blue}
        self.current_team = 'yellow'
        self.current_id = 0
        self.move_ball = False
        self.ball_pose = Pose()
    
    def model_states_callback(self, msg:ModelStates):
        if 'vss_ball' in msg.name:
            index = msg.name.index('vss_ball')
            self.ball_pose = msg.pose[index]
        
    def update(self):
        if self.move_ball: self.move_ball_fun()
        elif self.mode == 'direct': self.direct_mode()
        elif self.mode == 'diff': self.diff_mode()
        else: 
            rospy.logerr('Invalid mode')
            return
        
        if self.joystick.buttons['BACK']:
            self.reset_world()
        
        if self.joystick.buttons['X']:
            rospy.loginfo('Team blue')
            self.current_team = 'blue'
            self.current_id = 0
            self.move_ball = False
        if self.joystick.buttons['Y']:
            rospy.loginfo('Team yellow')
            self.current_team = 'yellow'
            self.current_id = 0
            self.move_ball = False

        if self.joystick.buttons['RB']:
            if self.next_robot_button: return
            if self.current_id < 2: self.current_id += 1
            else:  self.current_id = 0
            rospy.loginfo(f'Robot ID: {self.current_id}')
        
        if self.joystick.buttons['LB']:
            if self.prev_robot_button: return
            if self.current_id > 0: self.current_id -= 1
            elif self.current_id == 0: self.current_id = 2
            rospy.loginfo(f'Robot ID: {self.current_id}')
            
        if self.joystick.buttons['START']:
            self.move_ball = True
            
        if self.joystick.axes["PADV"] == -1:
            if self.light_down_button: return
            self.light -= 0.1
            self.light = max(self.light, 0.1)
            self.change_light(0)
            self.change_light(1)
        elif self.joystick.axes["PADV"] == 1:
            if self.light_up_button: return
            self.light += 0.1
            self.light = min(self.light, 1)
            self.change_light(0)
            self.change_light(1)
        
        # While the buttons are pressed do not update the values
        self.next_robot_button, self.prev_robot_button = self.joystick.buttons['RB'], self.joystick.buttons['LB']
        self.light_up_button, self.light_down_button = self.joystick.axes["PADV"] == 1, self.joystick.axes["PADV"] == -1
    
    def change_light(self, num):
        
        if num == 0:
            name = "spot_light_0"
            position = Point(x=-1.0, y=0.0, z=1.0)
            orientation = Quaternion(x=0.0, y=-0.3335, z=0.0, w=0.9428)
        else:
            name = "spot_light_1"
            position = Point(x=1.0, y=0.0, z=1.0)
            orientation = Quaternion(x=0.0, y=0.3335, z=0.0, w=0.9428)
        
        rospy.wait_for_service("/gazebo/set_light_properties")
            
        diffuse = ColorRGBA(r=self.light, g=self.light, b=self.light, a=255/255)
        specular = ColorRGBA(r=24/255, g=24/255, b=24/255, a=255/255)
        
        direction = Vector3(x=0.0, y=0.0, z=-1.0)
        pose = Pose(position=position, orientation=orientation)         
        
        msg = SetLightProperties()
        msg.light_name = name
        msg.cast_shadows = False
        msg.diffuse = diffuse
        msg.specular = specular
        msg.attenuation_constant = 0.5
        msg.attenuation_linear = 0.01
        msg.attenuation_quadratic = 0.001
        msg.direction = direction
        msg.pose = pose
        try:
            set_light = rospy.ServiceProxy("/gazebo/set_light_properties", SetLightProperties)
            resp = set_light(
                light_name=msg.light_name, 
                cast_shadows=msg.cast_shadows,
                diffuse=msg.diffuse,
                specular=msg.specular,
                attenuation_constant=msg.attenuation_constant,
                attenuation_linear=msg.attenuation_linear,
                attenuation_quadratic=msg.attenuation_quadratic,
                direction=msg.direction,
                pose=msg.pose
            )
            if not resp.success:
                rospy.logerr(f"Failed to set light properties: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        
    def move_ball_fun(self):
        self.ball_pose.position.y -= self.joystick.axes['LV']*0.01
        self.ball_pose.position.x += self.joystick.axes['LH']*0.01
        self.set_model_pose('vss_ball', self.ball_pose)
    
    def direct_mode(self):
        self.robots[self.current_team][self.current_id][0][0].publish(self.joystick.axes['LV']*self.vel)
        self.robots[self.current_team][self.current_id][1][0].publish(self.joystick.axes['RV']*self.vel)
        
    def diff_mode(self):
        V = self.joystick.axes['LV']*0.75
        W = self.joystick.axes['RH']*20.0
        
        wl = (2*V-self.b*W)/(60*self.r)*self.vel
        wr = (2*V+self.b*W)/(60*self.r)*self.vel
        
        wl, wr = np.clip([wl, wr], -self.vel, self.vel)

        self.robots[self.current_team][self.current_id][0][0].publish(wl)
        self.robots[self.current_team][self.current_id][1][0].publish(wr)
        
    def reset_world(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', EmptySrv)
            reset_world()
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
            
    def set_model_pose(self, model_name, pose):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = model_name
            state.pose = pose
            state.reference_frame = 'world'
            
            resp = set_state(state)
            if not resp.success:
                rospy.logerr(f"Failed to set pose for model {model_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
            
    
def main():
    rospy.init_node('joystick')
    # Available models: 'USB', 'Xbox360', 'XboxChinese', 'XboxOne'
    # Available modes: 'direct', 'diff'
    node_name = rospy.get_name()
    model = rospy.get_param(f'{node_name}/model')
    topic = rospy.get_param(f'{node_name}/topic')
    mode = rospy.get_param(f'{node_name}/mode')
    r = rospy.get_param(f'{node_name}/r')
    b = rospy.get_param(f'{node_name}/b')
    node = JoystickVSSS(model=model, topic=topic, mode=mode, r=r, b=b)
    rospy.spin()

if __name__ == '__main__':
    main()