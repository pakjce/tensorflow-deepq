import math
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from collections import defaultdict
from euclid import Circle, Point2, Vector2, LineSegment2

from ..utils import svg
from IPython.display import clear_output, display, HTML

class Agent(object):
    def __init__(self, jointAngle, speed):
        self.jointAngle = jointAngle
        self.speed = speed
        self.position = self.forward_kinematics()

    def move(self, deltaAngle):
        print 'TODO'

    def forward_kinematics(self):
        # self.jointAngle -> self.positon
        print 'TODO'


class RoboticEnvironment(object):
    def __init__(self, settings):
        self.settings = settings

        self.directions = [Vector2(*d) for d in [[1, 0], [0, 1], [-1, 0], [0, -1], [0.0, 0.0]]]
        self.num_actions = len(self.directions)

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
        print 'TODO'