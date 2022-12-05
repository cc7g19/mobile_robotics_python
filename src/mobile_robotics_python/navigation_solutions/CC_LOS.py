import netCDF4

import numpy as np
from numpy.linalg import norm

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.dates as mdate
from matplotlib import cm
from matplotlib.patches import Ellipse

from mpl_toolkits.mplot3d import Axes3D

from math import sqrt, cos, sin, atan2, pi

from scipy.interpolate import interp1d, interp2d, RectBivariateSpline


#Added Path and Point classes from sess6072_tutorials.py file. Might just copy in whole file somewhere?
#Defined desiredHeadingLOS, reached functions
#Created Perfect Vessel class as Ideal AUV definer
#Need to create a real vessel class to represent AUV with mass and forces

#DRAW BODY FRME FROM INTIAL TUTORIALS, USE INERTIAL BODUY FRAME, MAY HAVE HUGE AREA
#RESEARCH SLAM, PROBABILISTIC 

GRAVITY = 9.81
acceptance_radius = 10 #m?
los_radius = 15 #m?


class Point:
    def __init__(self, x, y):
        """Constructor. Set initial values."""
        self.x = x
        self.y = y

    def distanceTo(self, other):
        """Returns the L2 distance between the current and other points."""
        dx = self.x - other.x
        dy = self.y - other.y
        return sqrt(dx**2 + dy**2)

    def plot(self, marker="o", color="green"):
        """Plots the current point."""
        plt.plot(self.x, self.y, color=color, marker=marker)

    def plotLine(self, other):
        """Plots a line between the current and other point."""
        plt.plot([self.x, other.x], [self.y, other.y])

    def __str__(self):
        """Helper print function."""
        return "({},{})".format(self.x, self.y)


class Path:
    def __init__(self):
        """Constructor. Set empty list."""
        self.points = []

    def add(self, x, y):
        """Add a point to the list."""
        p = Point(x, y)
        self.addPoint(p)

    def addPoint(self, p):
        """Add a point to the list."""
        self.points.append(p)

    def __call__(self, i):
        """Returns the i-th point of the path."""
        return self.points[i]

    def __len__(self):
        return len(self.points)

    def __str__(self):
        """Helper print function."""
        msg = "Path:"
        for p in self.points:
            msg += "\n({},{})".format(p.x, p.y)
        return msg

    def plot(self, linestyle="dashed", color="black", marker=""):
        """Plots the path."""
        x0 = [p.x for p in self.points]
        x1 = [p.y for p in self.points]
        plt.plot(x0, x1, color=color, marker=marker, linestyle=linestyle)
        for p in self.points:
            p.plot()
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")


#Then create a start position using a point with coordinates x,y
#After that create a path (normally blank) and add points to it using .add(x,y)


#Then add LOS

def desiredHeadingLOS(position: Point, 
                      previous_waypoint: Point, 
                      current_waypoint: Point, 
                      los_radius=15.0, 
                      debug=False):
    # TIP: you can access a position/waypoints with .x/.y accessors.(I.e position.x, previous_waypoint.y)
    alpha = atan2(current_waypoint.y - previous_waypoint.y, current_waypoint.x - previous_waypoint.x)
    #Distance
    xdiff = previous_waypoint.x-position.x
    ydiff = previous_waypoint.y-position.y
    hypotenuse1 = np.sqrt(xdiff**2 + ydiff**2)
    #Along-track distance (los_s) and cross-track error (los_e)
    los_s = abs((position.x - previous_waypoint.x)*cos(alpha) + (position.y - previous_waypoint.y) * sin(alpha))
    los_e = -(np.sqrt((hypotenuse1**2 - los_s**2)))
    
    los_delta = 0.0 # this is correct as the base
    #Lookahead distance (los_delta) is always positive
    if los_radius > abs(los_e):
        los_delta = np.sqrt(los_radius**2 - los_e**2)
    #Orthogonal projection (where the blue line turns red)
    xproj = previous_waypoint.x + los_s * cos(alpha)
    yproj = previous_waypoint.y + los_s * sin(alpha)
    #Heading point
    losx = xproj + los_delta * cos(alpha) - position.x
    losy = yproj + los_delta * sin(alpha) - position.y
    #LOS heading
    los_heading = atan2(losy,losx)
    
    if los_heading < -pi:
        los_heading += pi
    elif los_heading > pi:
        los_heading -= pi
    
    if debug:
        print('Alpha: ', alpha)
        print('los_s: ', los_s)
        print('los_e: ', los_e)
        print('los_delta: ', los_delta)
        print('xproj: ', xproj)
        print('yproj: ', yproj)
        print('losx: ', losx)
        print('losy: ', losy)
        print('los_heading: ', los_heading*180/pi)
    
    # Return LOS heading angle
    return los_heading, Point(losx+position.x, losy+position.y)


def reached(current_position, waypoint, acceptance_radius):
    distance = current_position.distanceTo(waypoint)

    if distance <= acceptance_radius:
        d = True
    else:
        d = False
    return d


#Perfect vessels have no inertia and no speed/heading control, so instant changes
class PerfectVessel:
    def __init__(self, initial_position=Point(0,0), initial_heading=0):        
        self.position = initial_position
        self.heading = initial_heading
        self.speed = 0
        self.prev_speed = 0
        self.path = [self.position]
    
    def move(self, speed, heading, dt=0.5):
        # Copy the values
        self.speed = speed
        self.heading = heading
        
        # Wrap the heading
        if self.heading < -pi:
            self.heading += 2*pi
        elif self.heading > pi:
            self.heading -= 2*pi
        
        # Simple and perfect motion model
        new_x = self.position.x + cos(self.heading)*self.speed*dt
        new_y = self.position.y + sin(self.heading)*self.speed*dt
        
        # Create the point and update current position
        p = Point(new_x, new_y) 
        self.position = p
        self.path.append(p)
    def plot(self):
        self.position.plot(color='blue')

#Then provide Start point and heading, set ship = PerfectVessel(start_position, start_heading)
#Input cruise speed and acceptance radius, provide current waypoint index