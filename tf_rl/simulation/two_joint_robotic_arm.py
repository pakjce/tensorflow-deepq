import math
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from collections import defaultdict
from euclid import Circle, Point2, Vector2, LineSegment2

from ..utils import svg
from IPython.display import clear_output, display, HTML
import numpy as np
from collections import deque

class TargetObject(object):
    def __init__(self, settings, position=None):
        self.settings = settings
        self.radius = int(self.settings['target']['radius'])
        if(position == None):
            self.rand_replace()
        else:
            self.position = position

    def rand_replace(self):
        Xmax = self.settings['world_size'][0]
        Ymax = self.settings['world_size'][1]
        x0 = self.settings['agent']['origin'][0]
        y0 = self.settings['agent']['origin'][1]
        l = self.settings['agent']['jointLengths']
        r = l[0] + l[1]
        r2 = r * r
        while(True):
            X = np.random.random_sample(2)
            x = X[0] * Xmax
            y = X[1] * Ymax
            t = np.power(x-x0, 2) + np.power(y-y0, 2)
            if(t<= r2):
                break
        self.position = [int(x), int(y)]

    def check_reached(self, agent_position):
        t_dist = np.power(self.position[0] - agent_position[0], 2) + np.power(self.position[1] - agent_position[1], 2)
        t_dist = np.sqrt(t_dist)

        if(t_dist <= self.radius):
            return 1
        else:
            return 0

    def move(self, position):
        self.position = position

    def draw(self, scene):
        """Return svg object for this item."""
        color = self.settings['target']["color"]
        scene.add(svg.Circle(self.position + Point2(10, 10), self.radius, color=color))

class Agent(object):
    def __init__(self, settings, jointAngle=None):

        self.speed = [0, 0]

        self.settings = settings
        self.jointLength = self.settings['agent']['jointLengths']
        self.origin = self.settings['agent']['origin']
        self.end_point_traj = deque()

        if (jointAngle == None):
            self.rand_replace()
        else:
            self.jointAngle = jointAngle

        # position: 3 x 2 vector
        self.forward_kinematics()

    def rand_replace(self):
        lims = self.settings['agent']['jointLimits']

        X = np.random.random_sample(2)
        X[0] = X[0] * (lims[0][1] - lims[0][0]) + lims[0][0]
        X[1] = X[1] * (lims[1][1] - lims[1][0]) + lims[1][0]

        self.jointAngle = X * np.pi / 180.0

    def set_speed(self, speed_change):
        lims = self.settings['agent']['speedLimits'] # (deg/sec)
        curr_speed = self.speed
        curr_speed[0] = curr_speed[0] + speed_change[0]
        curr_speed[1] = curr_speed[1] + speed_change[1]

        # (deg/sec) -> (rad/sec)
        self.speed[0] = min(max(curr_speed[0], lims[0][0]), lims[0][1]) * np.pi / 180.0
        self.speed[1] = min(max(curr_speed[1], lims[1][0]), lims[1][1]) * np.pi / 180.0

    def stop(self):
        self.speed = [0, 0]

    def move(self, dt):
        """Move as if dt seconds passed"""
        lims = self.settings['agent']['jointLimits']
        # print '[move] curr joint Angle:'
        # print self.jointAngle
        # print '[move] curr speed:'
        # print self.speed

        J = self.jointAngle + dt * np.array(self.speed)
        self.jointAngle[0] = min(max(J[0], lims[0][0]), lims[0][1])
        self.jointAngle[1] = min(max(J[1], lims[1][0]), lims[1][1])
        self.forward_kinematics()

    def step(self, dt):
        """Move"""
        self.move(dt)

    def forward_kinematics(self):
        # self.jointAngle -> self.positon
        self.position = np.zeros((3,2))
        # origin
        self.position[0, :] = self.origin
        # joint 1
        self.position[1, 0] = self.position[0, 0] + self.jointLength[0] * np.cos(self.jointAngle[0])
        self.position[1, 1] = self.position[0, 1] + self.jointLength[0] * np.sin(self.jointAngle[0])

        # joint 2 (endpoint)
        self.position[2, 0] = self.position[1, 0] + self.jointLength[1] * np.cos(self.jointAngle[0] + self.jointAngle[1])
        self.position[2, 1] = self.position[1, 1] + self.jointLength[1] * np.sin(self.jointAngle[0] + self.jointAngle[1])

        self.end_point_traj.append(self.position[2, :])

    def draw(self, scene):
        """Return svg object for the arm """
        scene.add(svg.Line(start=[self.position[0, 0], self.position[0, 1]], end=[self.position[1, 0], self.position[1, 1]], thickness=3))
        scene.add(svg.Line(start=[self.position[1, 0], self.position[1, 1]], end=[self.position[2, 0], self.position[2, 1]], thickness=3))

        scene.add(svg.Circle(center=self.position[0, :], radius=3, color='gray'))
        scene.add(svg.Circle(center=self.position[1, :], radius=3, color='gray'))

        # draw trajectory
        X = list(self.end_point_traj)
        for i in range(1, len(self.end_point_traj)):
            x1 = X[i-1]
            x2 = X[i]
            scene.add(svg.Line(start=x1, end=x2, thickness=1, color='red'))

        scene.add(svg.Circle(center=self.position[2, :], radius=3, color='blue'))



class RoboticEnvironment(object):
    def __init__(self, settings):
        self.settings = settings
        self.num_actions = 8
        self.observation_size = 10

        self.agent = Agent(settings=settings)
        self.target = TargetObject(settings=settings)
        self.size = self.settings["world_size"]

    def reset(self):
        self.agent = Agent(settings=self.settings)
        self.target = TargetObject(settings=self.settings)

    def perform_action(self, action_id):
        # actions
        # 0: stop
        # 1: [0, 0] keep
        # 2: [+, 0]
        # 3: [+, -]
        # 4: [+, +]
        # 5: [-, 0]
        # 6: [-, -]
        # 7: [-, +]
        assert 0 <= action_id < self.num_actions

        amt = float(self.settings['action']['speed_amount'])

        if (action_id == 0):
            self.agent.stop()
        elif (action_id == 1):
            self.agent.set_speed([0, 0])
        elif (action_id == 2):
            self.agent.set_speed([amt, 0])
        elif (action_id == 3):
            self.agent.set_speed([amt, -amt])
        elif (action_id == 4):
            self.agent.set_speed([amt, amt])
        elif (action_id == 5):
            self.agent.set_speed([-amt, 0])
        elif (action_id == 6):
            self.agent.set_speed([-amt, -amt])
        elif (action_id == 7):
            self.agent.set_speed([-amt, amt])


    def step(self, dt):
        self.agent.step(dt)

    def observe(self):
        """Return observation vector.
        0-1: agent end-point (x,y) # pixel
        2-3: agent speed (j1, j2) # rad/s
        4-5: agent joint status # rad
        6-7: goal position (x, y) # pixel
        8: reached (0 or 1)
        9: hop count / MAX_TRIAL
        """
        observation = np.zeros(self.observation_size)

        # agent end-point
        observation[0] = self.agent.position[2][0] / float(self.settings['world_size'][0])
        observation[1] = self.agent.position[2][1] / float(self.settings['world_size'][1])

        # agent speed
        speed_limits = self.settings['agent']['speedLimits']
        observation[2] = (self.agent.speed[0]-speed_limits[0][0]) / float(speed_limits[0][1]-speed_limits[0][0])
        observation[3] = (self.agent.speed[1] - speed_limits[1][0]) / float(speed_limits[1][1] - speed_limits[1][0])

        # agent joint angle
        joint_limits = self.settings['agent']['jointLimits']
        observation[4] = (self.agent.jointAngle[0] - joint_limits[0][0]) / float(joint_limits[0][1] - joint_limits[0][0])
        observation[5] = (self.agent.jointAngle[1] - joint_limits[1][0]) / float(joint_limits[1][1] - joint_limits[1][0])

        # goal position
        observation[6] = self.target.position[0] / float(self.settings['world_size'][0])
        observation[7] = self.target.position[1] / float(self.settings['world_size'][1])

        # reached
        observation[8] = self.target.check_reached(self.agent.position[2])

        # hop count
        observation[9] = len(self.agent.end_point_traj) / float(self.settings['maximum_trial'])

        return observation

    def collect_reward(self):
        # 1. give a reward when target are reached
        total_reward = self.target.check_reached(self.agent.position[2]) * float(self.settings['target']['reward'])

        # 2. discount for hop count

        discount = 1.0 - len(self.agent.end_point_traj) / float(self.settings['maximum_trial'])

        total_reward = total_reward * discount

        return total_reward

    def _repr_html_(self):
        return self.to_html()

    def to_html(self, stats=[]):
        scene = svg.Scene((self.size[0] + 20, self.size[1] + 20 + 20 * len(stats)))
        scene.add(svg.Rectangle((10, 10), self.size))

        self.target.draw(scene)

        self.agent.draw(scene)
        scene.add(svg.Text((10, 20), '[%.0f %.0f] [%.0f %.0f]' % (self.agent.position[1,0], self.agent.position[1, 1], self.agent.position[2, 0], self.agent.position[2, 1]), 15))


        return scene

    def setup_draw(self):
        """
        An optional method to be triggered in simulate(...) to initialise
        the figure handles for rendering.
        simulate(...) will run with/without this method declared in the simulation class
        As we are using SVG strings in KarpathyGame, it is not curently used.
        """
        pass

    def draw(self, stats=[]):
        """
        An optional method to be triggered in simulate(...) to render the simulated environment.
        It is repeatedly called in each simulated iteration.
        simulate(...) will run with/without this method declared in the simulation class.
        """
        clear_output(wait=True)
        svg_html = self.to_html(stats)
        display(svg_html)