#!/usr/bin/python3
import rospy, rospkg, sys, numpy as np
sys.dont_write_bytecode = True

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

from std_srvs.srv import Empty as EmptySrv
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
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
        
        self.mode = mode
        self.r, self.b = r, b
        
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
            if self.current_id < 2: self.current_id += 1
            else:  self.current_id = 0
            rospy.loginfo(f'Robot ID: {self.current_id}')
        
        if self.joystick.buttons['LB']:
            if self.current_id > 0: self.current_id -= 1
            elif self.current_id == 0: self.current_id = 2
            rospy.loginfo(f'Robot ID: {self.current_id}')
            
        if self.joystick.buttons['START']:
            self.move_ball = True
        
    def move_ball_fun(self):
        self.ball_pose.position.y -= self.joystick.axes['LV']*0.01
        self.ball_pose.position.x += self.joystick.axes['LH']*0.01
        # self.ball_pose.position.z = 0.03
        # self.ball_pose.orientation.w = 1.0
        self.set_model_pose('vss_ball', self.ball_pose)
    
    def direct_mode(self):
        self.robots[self.current_team][self.current_id][0][0].publish(self.joystick.axes['LV']/self.r)
        self.robots[self.current_team][self.current_id][1][0].publish(self.joystick.axes['RV']/self.r)
        
    def diff_mode(self):
        V = self.joystick.axes['LV']
        W = self.joystick.axes['RH']
        
        if abs(V)+abs(W) > 1: 
            V = V/(abs(V)+abs(W))
            W = W/(abs(V)+abs(W))
        
        wl = (V-W*self.b/2)/self.r
        wr = (V+W*self.b/2)/self.r
        
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