#!/usr/bin/env python

# Original code (without cruise control features) based on Clearpath Husky teleop
# @ hydro devel (now depreciated).  Not forked for simplicity since it's just one file.

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Teleop:
    def __init__(self):
        rospy.init_node('husky_teleop')

        # -- Normal parameters -- 

        # Scales that convert raw joystick values to linear/angular velocities.
        self.turn_scale = rospy.get_param('~turn_scale')
        self.drive_scale = rospy.get_param('~drive_scale')

        # -- Normal buttons -- 
        
        # Deadman button must being pressed to use command velocity from joystick.
        self.deadman_button = rospy.get_param('~deadman_button', 0)
        
        # Planner button will pass on a 'planned' velocity if deadman isn't being pressed.
        self.planner_button = rospy.get_param('~planner_button', 1)
        
        # -- Cruise control parameters -- 
        
        # Max/min that the cruise speed can be set to. In m/s.
        self.cruise_max = rospy.get_param('~cruise_max')        
        
        # How much to increment/decrement speed each time button is pressed. In m/s.
        self.cruise_change = rospy.get_param('~cruise_change')
        
        # The velocity the preset button will set the cruise to. In m/s.
        self.preset_velocity = rospy.get_param('~preset_velocity')
        
        # -- Cruise control buttons (only affects linear speed, not turning) -- 
        
        # Set cruise speed to current robot speed.
        self.cruise_set_button = rospy.get_param('~cruise_set_button', 2)
        
        # Exit cruise mode and goes back to normal operation.
        self.cruise_clear_button = rospy.get_param('~cruise_clear_button', 3)
        
        # Increment / decrement cruise speed.
        self.cruise_inc_button = rospy.get_param('~cruise_inc_button', 4)
        self.cruise_dec_button = rospy.get_param('~cruise_dec_button', 5)
        
        # Return to speed that was last set before 'clearing' out of cruise mode.
        self.cruise_resume_button = rospy.get_param('~cruise_resume_button', 6)
        
        # Set cruise speed to a pre-defined value.
        self.cruise_preset_button = rospy.get_param('~cruise_preset_button', 7)
        
        self.cruise = CruiseControl(self.cruise_max)
        
        self.cmd = None
        self.joy = Joy()
        cmd_pub = rospy.Publisher('cmd_vel', Twist)

        rospy.Subscriber("joy", Joy, self.callback)
        rospy.Subscriber("plan_cmd_vel", Twist, self.planned_callback)
        self.planned_motion = Twist()
        
        rate = rospy.Rate(rospy.get_param('~hz', 20))
        
        while not rospy.is_shutdown():
            rate.sleep()
            if self.cmd:
                cmd_pub.publish(self.cmd)

    def planned_callback(self, data):
        """ Handle incoming Twist command from a planner.
        Manually update motion planned output if the buttons
        are in the right state """
        self.planned_motion = data 
        if self.joy.buttons[self.deadman_button] == 0 and\
           self.joy.buttons[self.planner_button] == 1:
            self.cmd = self.planned_motion

    def callback(self, data):
        """ Receive joystick data, formulate Twist message.
        Use planner if a secondary button is pressed """
        self.joy = data
        cmd = Twist()
        cmd.linear.x = data.axes[1] * self.drive_scale
        cmd.angular.z = data.axes[0] * self.turn_scale

        self.update_cruise_control(cmd.linear.x)

        if data.buttons[self.deadman_button] == 1:
            self.cmd = cmd
        elif data.buttons[self.planner_button] == 1:
            self.cmd = self.planned_motion
        elif self.cruise.enabled:
            self.cmd.linear.x = self.cruise.velocity
            self.cmd = cmd
        else:
            self.cmd = None
            
    def update_cruise_control(self, current_vel):
        """Update linear 'x' velocity of cmd message based on the current cruise control settings.
          If not in cruise control mode then cmd will be unchanged. Before calling the cmd.linear.x
          value should be the user desired velocity."""
        
        if data.buttons[self.cruise_clear_button] == 1:
            self.cruise.clear()
            
        elif data.buttons[self.cruise_set_button] == 1:
            self.cruise.set(current_vel)
            
        elif data.buttons[self.cruise_preset_button] == 1:
            self.cruise.set(self.preset_velocity)
            
        elif data.buttons[self.cruise_resume_button] == 1:
            self.cruise.resume()

        # if holding both buttons then will cancel out.
        if data.buttons[self.cruise_inc_button] == 1:
            self.cruise.increment(self.cruise_change)
            
        if data.buttons[self.cruise_dec_button] == 1:
            self.cruise.increment(-self.cruise_change)
        
            
class CruiseControl(object):
    
    def __init__(self, max_velocity):
        self.velocity_ = 0
        self.max_velocity = max_velocity
        self.enabled = False # true if actively in cruise control mode
        
    @property
    def velocity(self):
        return self.velocity if self.enabled else 0
        
    def increment(self, vel_increment):
        if self.enabled:
            self.set(self.velocity + vel_increment)
        
    def set(self, new_velocity):
        self.velocity = self.limit_velocity(new_velocity)
        self.enabled = True
        
    def resume(self):
        self.enabled = True
        
    def clear(self):
        self.enabled = False

    def limit_velocity(self, velocity):
        if velocity > self.max_velocity:
            return self.max_velocity
        if velocity < -self.max_velocity:
            return -self.max_velocity
        return velocity
        

if __name__ == "__main__": Teleop()