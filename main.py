"""
    robot - makex
    ชื่อค่อยคิด เป็นปัญหาของกูในอนาคต    
"""

import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class
from mbuild.smart_camera import smart_camera_class
from mbuild.ranging_sensor import ranging_sensor_class
from mbuild.smartservo import smartservo_class
from mbuild import power_manage_module

# Utility class, this is a class that contains a bunch of useful functions
class util:
    def restrict(val, min, max):
        if val > max:
            return max
        if val < min:
            return min
        return val    

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired value (target)
        self.integral = 0  # Sum of errors over time
        self.previous_error = 0  # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        """ Update the target setpoint for the PID controller """
        self.setpoint = setpoint
        self.integral = 0  # Reset the integral to avoid wind-up
        self.previous_error = 0  # Reset previous error to avoid a large derivative spike

class dc_motor:
    def __init__(self, motor_port):
        self.motor_port = motor_port
        
    def set_power(self, power):
        power_expand_board.set_power(self.motor_port, power)
        
    def enable(self, inverse=False):
        power_expand_board.set_power(self.motor_port, -100 if inverse else 100)
        
    def disable(self):
        power_expand_board.set_power(self.motor_port, 0)
        
class mecanum_drive:
    
    def __init__(self, left_front, left_right, front_left, front_right, deadzone=20):
        self.left_front = encoder_motor_class(left_front, "INDEX1")
        self.left_right = encoder_motor_class(left_right, "INDEX1")
        self.front_left = encoder_motor_class(front_left, "INDEX1")
        self.front_right = encoder_motor_class(front_right, "INDEX1")
        self.deadzone = deadzone
        
    def motor_drive(self, left_front_speed, left_right_speed, front_left_speed, front_right_speed):
        self.left_front.set_speed(left_front_speed)
        self.left_right.set_speed(left_right_speed)
        self.front_left.set_speed(front_left_speed)
        self.front_right.set_speed(front_right_speed)
    
    def drive(self, velocity_x, velocity_y, angular_velocity):
        # Use deadzone to prevent motor from moving when joystick is not touched
        if math.fabs(velocity_x) < math.fabs(self.deadzone):
            velocity_x = 0
        if math.fabs(velocity_y) < math.fabs(self.deadzone):
            velocity_y = 0
        if math.fabs(angular_velocity) < math.fabs(self.deadzone):
            angular_velocity = 0
    
        # Calculation for the wheel speed
        # Visit https://github.com/neumann-lab/holonomic-mecanum/blob/main/th.md for the formula
        vFL = velocity_x + velocity_y + angular_velocity
        vFR = -(velocity_x) + velocity_y - angular_velocity
        vBL = -(velocity_x) + velocity_y + angular_velocity
        vBR = velocity_x + velocity_y - angular_velocity
        
        vFL = util.restrict(vFL, -180, 180)
        vFR = util.restrict(vFR, -180, 180)
        vBL = util.restrict(vBL, -180, 180)
        vBR = util.restrict(vBR, -180, 180)
        
        self.motor_drive(vFL, vFR, vBL, vBR)

class manual:
    
    def __init__(self, mode):
        self.mode = mode
    
    def run(self):
        pass

class automatic:
    
    def __init__(self):
        pass
    
    def run(self):
        pass

class robot:
    def __init__(self, movement: mecanum_drive, manual_stage: manual, automatic_stage: automatic):
        self.movement = movement
        self.manual = manual_stage
        self.automatic = automatic_stage
    
    def joystick_control(self):
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            self.movement.drive(gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), gamepad.get_joystick("Rx"))
        else:
            self.movement.motor_drive(0, 0, 0, 0)
        
    def handle_manual_stage(self):
        self.joystick_control()
        self.manual.run()
    
    def handle_automatic_stage(self):
        self.automatic.run()

robot = robot(mecanum_drive("M1", "M2", "M3", "M4"), manual("shoot"), automatic())

while True:
    if power_manage_module.is_auto_mode():
        robot.handle_automatic_stage()
        while power_manage_module.is_auto_mode():
            pass
    else:
        robot.handle_manual_stage()