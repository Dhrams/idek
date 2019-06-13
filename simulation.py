import matplotlib
import numpy as np
import math
import gym
from gym import spaces, logger
from gym.utils import seeding



class Simulation(gym.Env):
    def __init__(self):
        self.motor_mass = 0.06
        self.grav = 9.81
        self.pole_mass = 0.04
        self.polelen = 0.2
        self.tau = 0.001
        self.upperbound = 30
        self.lowerbound = -60
        self.state = None
        self.action_space = spaces.Discrete(2)
        self.seed()

    def dynamics(self, PWM):
        state = self.state
        theta, theta_dot = state
        costheta = math.cos(theta)
        sintheta = math.sin(theta)
        thetadd = ((PWM**2)*self.polelen - (self.polemass/2 + self.motormass)*self.gravity*self.length* costheta)/((self.polemass/3 + self.motormass)*self.length**2)
        theta = theta + self.tau * theta_dot
        theta_dot = theta_dot + self.tau * thetadd
        self.state = (theta,theta_dot)
        done =  theta < self.theta_threshold_radians_lower \
                or theta > self.theta_threshold_radians_upper
        done = bool(done)

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        return np.array(self.state), reward, done, {}

    def reset(self):
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(2,))
        self.steps_beyond_done = None
        return np.array(self.state)
