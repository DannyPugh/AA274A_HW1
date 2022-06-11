import numpy as np
from utils import wrapToPi

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1, k2, k3, V_max=0.5, om_max=1):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.V_max = V_max
        self.om_max = om_max

    def load_goal(self, x_g, y_g, th_g):
        """ Loads in a new goal position """
        self.x_g = x_g
        self.y_g = y_g
        self.th_g = th_g

    def compute_control(self, x, y, th, t):
        """
        Inputs:
            x,y,th: Current state
            t: Current time (you shouldn't need to use this)
        Outputs: 
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
        x_g = self.x_g
        y_g = self.y_g
        th_g = self.th_g

        #xp = x - x_g
        #yp = y - y_g

        xp = x_g - x
        yp = y_g - y

        rho = np.sqrt(xp**2 + yp**2)
        alpha = wrapToPi(np.arctan2(yp, xp)-th)
        #alpha = wrapToPi(np.pi-(th - np.arctan2(yp, xp)))
        delta = wrapToPi(np.arctan2(yp, xp)-th_g)


        k1 = self.k1
        k2 = self.k2
        k3 = self.k3

        #??? How to impliment np.sinc(alpha) ???
        V = k1*rho*np.cos(alpha)
        om = k2*alpha + k1*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha + k3*delta)
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om
