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

    def move(self, position):
        self.position = position

    def draw(self, scene):
        """Return svg object for this item."""
        color = self.settings['target']["color"]
        scene.add(svg.Circle(self.position + Point2(10, 10), self.radius, color=color))

class Agent(object):
    def __init__(self, speed, settings, jointAngle=None):

        self.speed = speed

        self.settings = settings
        self.jointLength = self.settings['agent']['jointLengths']
        self.origin = self.settings['agent']['origin']

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


    def move(self, dt):
        """Move as if dt seconds passed"""

        print 'TODO'

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

    def draw(self, scene):
        """Return svg object for the arm """
        scene.add(svg.Line(start=[self.position[0, 0], self.position[0, 1]], end=[self.position[1, 0], self.position[1, 1]]))
        scene.add(svg.Line(start=[self.position[1, 0], self.position[1, 1]], end=[self.position[2, 0], self.position[2, 1]]))

        scene.add(svg.Circle(center=self.position[0, :], radius=3, color='gray'))
        scene.add(svg.Circle(center=self.position[1, :], radius=3, color='gray'))
        scene.add(svg.Circle(center=self.position[2, :], radius=3, color='blue'))


class RoboticEnvironment(object):
    def __init__(self, settings):
        self.settings = settings

        self.directions = [Vector2(*d) for d in [[1, 0], [0, 1], [-1, 0], [0, -1], [0.0, 0.0]]]
        self.num_actions = len(self.directions)

        self.agent = Agent(settings=settings, speed=[0,0])
        self.target = TargetObject(settings=settings)
        self.size = self.settings["world_size"]

    def perform_action(self, action_id):
        assert 0 <= action_id < self.num_actions

    def step(self, dt):
        print 'TODO'

    def observe(self):
        """Return observation vector. For all the observation directions it returns representation
        of the closest object to the hero - might be nothing, another object or a wall.
        Representation of observation for all the directions will be concatenated.
        """
        print 'TODO'

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