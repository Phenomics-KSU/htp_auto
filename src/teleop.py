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
        self.turn_scale = rospy.get_param('~turn_scale', 1)
        self.drive_scale = rospy.get_param('~drive_scale', 1)

        # -- Normal buttons -- 
        
        # Deadman button must being pressed to use command velocity from joystick.
        self.deadman_button = rospy.get_param('~deadman_button', 2) # x
        
        # Planner button will pass on a 'planned' velocity if deadman isn't being pressed.
        self.planner_button = rospy.get_param('~planner_button', 1) # b
        
        # -- Cruise control parameters -- 
        
        # Max/min that the cruise speed can be set to. In m/s.
        self.cruise_max = rospy.get_param('~cruise_max', 1)        
        
        # How much to increment/decrement speed each time button is pressed. In m/s.
        self.cruise_change = rospy.get_param('~cruise_change', 0.05)
        
        # The velocity the preset button will set the cruise to. In m/s.
        self.preset_velocity = rospy.get_param('~preset_velocity', 0.25)
        
        # -- Cruise control buttons (only affects linear speed, not turning) -- 
      
        # Exit cruise mode and goes back to normal operation.
        self.cruise_clear_button1 = rospy.get_param('~cruise_clear_button1', 0) # 'A' button
        self.cruise_clear_button2 = rospy.get_param('~cruise_clear_button2', 9) # left stick click in 
        self.cruise_clear_button3 = rospy.get_param('~cruise_clear_button3', 10) # right stick click in
        
        # Increment / decrement cruise speed.
        self.cruise_change_axis = rospy.get_param('~cruise_change_axis', 7) # dpad up/down
        
        # Enable cruise control.
        self.cruise_enable_button1 = rospy.get_param('~cruise_enable_button1', 5) # rb
        self.cruise_enable_button2 = rospy.get_param('~cruise_enable_button2', 4) # lb
        
        # Return to speed that was last set before 'clearing' out of cruise mode.
        self.cruise_resume_button = rospy.get_param('~cruise_resume_button', 6) # back button 
        
        # Set cruise speed to a pre-defined value.
        self.cruise_preset_button = rospy.get_param('~cruise_preset_button', 7) # start button
        
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
        """
        Handle incoming Twist command from a planner.
        Manually update motion planned output if the buttons
        are in the right state
        """
        self.planned_motion = data 
        if self.joy.buttons[self.deadman_button] == 0 and\
           self.joy.buttons[self.planner_button] == 1:
            self.cmd = self.planned_motion

    def callback(self, data):
        """
        Receive joystick data, formulate Twist message.
        Use planner if a secondary button is pressed
        """
        self.joy = data
        cmd = Twist()
        cmd.linear.x = data.axes[1] * self.drive_scale
        cmd.angular.z = data.axes[0] * self.turn_scale

        self.update_cruise_control(data, cmd.linear.x)

        if data.buttons[self.deadman_button] == 1:
            self.cmd = cmd
        elif data.buttons[self.planner_button] == 1:
            self.cmd = self.planned_motion
        elif self.cruise.enabled:
            cmd.linear.x = self.cruise.velocity
            self.cmd = cmd
        else: 
            # No active command.
            cmd.linear.x = 0
            cmd.angular.z = 0
            self.cmd = cmd
            
    def update_cruise_control(self, data, current_vel):
        """
        Update linear 'x' velocity of cmd message based on the current cruise control settings.
        If not in cruise control mode then cmd will be unchanged. Before calling the cmd.linear.x
        value should be the user desired velocity.
        """
        if self.cruise_clear_button_active(data):
            self.cruise.clear()
            
        elif self.cruise_enable_button_active(data):
            # Go into normal cruise control
            self.cruise.set(current_vel)
        
        elif data.buttons[self.cruise_resume_button] == 1:
            # Make sure we already had a non-zero velocity to return
            # to so user doesn't accidentaly end up in cruise mode.
            if self.cruise.last_velocity != 0:
                self.cruise.resume()
        
        elif data.buttons[self.cruise_preset_button] == 1:
            # Make sure already in cruise mode to avoid user accidently
            # going straight to preset velocity.
            if self.cruise.enabled:
                self.cruise.set(self.preset_velocity)
                
        if data.axes[self.cruise_change_axis] == 1:
            self.cruise.increment(self.cruise_change)
            
        if data.axes[self.cruise_change_axis] == -1:
            self.cruise.increment(-self.cruise_change)
        
    def cruise_enable_button_active(self ,data):
        """Return true only if all of the enable buttons are pressed."""
        return (data.buttons[self.cruise_enable_button1] == 1 and
                data.buttons[self.cruise_enable_button2] == 1)
        
    def cruise_clear_button_active(self ,data):
        """Return true if any of the clear butons are pressed."""
        return (data.buttons[self.cruise_clear_button1] == 1 or
                data.buttons[self.cruise_clear_button2] == 1 or
                data.buttons[self.cruise_clear_button3] == 1)
           
class CruiseControl(object):
    
    def __init__(self, max_velocity):
        self.velocity_ = 0
        self.max_velocity = max_velocity
        self.enabled = False # true if actively in cruise control mode
        
    @property
    def velocity(self):
        return self.velocity_ if self.enabled else 0
    
    @property
    def last_velocity(self):
        return self.velocity_ 
        
    def increment(self, vel_increment):
        if self.enabled:
            self.set(self.velocity + vel_increment)
        
    def set(self, new_velocity):
        self.velocity_ = self.limit_velocity(new_velocity)
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