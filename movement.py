from math import pi
from ev3dev2.motor import SpeedPercent

class RobotMovement:
    def __init__(self, motor_left, motor_right, wheel_diameter, base_distance, speed, transmission_ratio):
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.wheel_diameter = wheel_diameter
        self.base_distance = base_distance
        self.speed = speed * transmission_ratio
        self.transmission_ratio = transmission_ratio

    def turn(self, degrees):
        if degrees == 0:
            return 
        wheel_circunference = pi * self.wheel_diameter
        base_circunference = pi * self.base_distance

        turn_distance = (base_circunference / (360 / abs(degrees)))
        
        wheel_turn_degrees = ((turn_distance / wheel_circunference) * 360) * self.transmission_ratio
        
        direction = -1 if degrees > 0 else 1  # Clockwise: degrees > 0; anticlockwise: degrees < 0
        
        self.motor_left.reset()
        self.motor_right.reset()

        
        self.motor_left.on_for_degrees(SpeedPercent(-self.speed), wheel_turn_degrees * direction, brake=True, block=False)
        self.motor_right.on_for_degrees(SpeedPercent(-self.speed), -wheel_turn_degrees * direction, brake=True, block=True)

    def move(self, distance):
        wheel_circunference = pi * self.wheel_diameter
        advance_degrees = ((distance / wheel_circunference) * 360) * self.transmission_ratio
        
        self.motor_left.reset()
        self.motor_right.reset()
        
        self.motor_left.on_for_degrees(SpeedPercent(-self.speed), advance_degrees, brake=True, block=False)
        self.motor_right.on_for_degrees(SpeedPercent(-self.speed), advance_degrees, brake=True, block=True)

    def stop(self):
        self.motor_left.off()
        self.motor_right.off()