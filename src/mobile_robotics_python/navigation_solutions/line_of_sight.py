import weakref

from mobile_robotics_python.messages import RobotStateMessage, SpeedRequestMessage

from . import NavigationSolutionBase


class LineOfSight(NavigationSolutionBase):
    def __init__(self, parameters, parent):
        """Initialise the algorithm.

        Parameters
        ----------
        parameters : dict
            Configuration parameters for the naive rotate move algorithm.
        """
        self.parameters = parameters
        self.orientation_threshold = parameters["orientation_threshold"] 
        self.rotation_speed = parameters["rotation_speed"]
        self.linear_speed = parameters["linear_speed"]
        if parent is not None:
            self._parent = weakref.ref(parent)
        
    def distanceTo(self, other):
        """Returns the L2 distance between the current and other points."""
        dx = self.parameters.x - other.x
        dy = self.parameters.y - other.y
        return sqrt(dx**2 + dy**2) 
    
#HERE I NEED TO CHECK FOR IF THIS .X AND .Y WORKS OR IF I NEED TO USE self.parameters[x] 
#WHERE X IS ITS POSITION IN THE LIST/TUPLE USED WHEN REPORITNG FOR EACH TIMESTAMP


    def desiredHeadingLOS(self,
                          current_position: RobotStateMessage, 
                          previous_waypoint: RobotStateMessage, 
                          next_waypoint: RobotStateMessage, 
                          los_radius=0.1, 
                          debug=False):
        
        #I'M CURRENTLY ASSUMING THE WAY IT RUNS WILL LOOP THIS FUNCTION ON ITS OWN, SO ITLL PROVOIDE ITS OWN UPDATED INPUTS
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
    
        #Return LOS heading angle
        return los_heading #, Point(losx+position.x, losy+position.y)

    def compute_request(self,
        current_position: RobotStateMessage,
        previous_waypoint: RobotStateMessage,
        next_waypoint: RobotStateMessage,
    ) -> SpeedRequestMessage:
        """Computes the speed request to move from the current position to the desired position.

        Parameters
        ----------
        current_position : RobotStateMessage
            Current position.
        previous_waypoint : RobotStateMessage
            Previous waypoint.
        next_waypoint : RobotStateMessage
            Next waypoint.

        Returns
        -------
        SpeedRequestMessage
            Speed request.
        """
        # Your code starts here:
        msg = SpeedRequestMessage()
        desired_theta = self.desiredHeadingLOS(self, current_position, previous_waypoint, next_waypoint)
        diff_theta = desired_theta - current_positon.yaw_rad
        
        if abs(diff_theta) > self.orientation_threshold:
            msg.wz_radps = self.rotation_speed * sign_with_zero(diff_theta)
        else:
            msg.wz_radps = 0

        # You will need to change the contents of this message with the required values
        msg.vx_mps = self.linear_speed
        msg.stamp_s = get_utc_stamp()
        # At the end of the function, return the speed request
        #print msg
        return msg
