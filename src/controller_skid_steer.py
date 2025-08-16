import time
import os
import sys
import argparse
sys.path.append("/home/astrobotic/Documents/SDK/MotorControllers/roboclaw/Libraries/Python/roboclaw_python")
from pyPS4Controller.controller import Controller
from roboclaw_3 import Roboclaw

rc = Roboclaw("/dev/ttyS0", 38400)
rc.Open()
address1 = 0x80
address2 = 0x81

class JoystickController(Controller):
    def __init__(self, maxSpeed, deadZone, **kwargs):
        super().__init__(**kwargs)
        self.maxSpeed = maxSpeed
        self.deadZone = deadZone

    def mapSpeed(self, value):
        if abs(value) < self.deadZone:
            return 0
        return int((abs(value) / 32767.0) * self.maxSpeed)

    def on_L3_up(self, value):
        speed = self.mapSpeed(value)
        rc.SpeedM1(address1, -speed)
        rc.SpeedM2(address2, -speed)

    def on_L3_down(self, value):
        speed = self.mapSpeed(value)
        rc.SpeedM1(address1, speed)
        rc.SpeedM2(address2, speed)

    def on_R3_up(self, value):
        speed = self.mapSpeed(value)
        rc.SpeedM2(address1, speed)
        rc.SpeedM1(address2, speed)

    def on_R3_down(self, value):
        speed = self.mapSpeed(value)
        rc.SpeedM2(address1, -speed)
        rc.SpeedM1(address2, -speed)

    def on_x_press(self):
        print("X pressed - stopping all motors.")
        rc.SpeedM1(address1, 0)
        rc.SpeedM2(address1, 0)
        rc.SpeedM1(address2, 0)
        rc.SpeedM2(address2, 0)
        os._exit(0)

class TriggerController(Controller):
    def __init__(self, maxSpeed, **kwargs):
        super().__init__(**kwargs)
        self.speed = maxSpeed
        self.left = 0
        self.right = 0

    def updateMotors(self):
        if self.left == 1:
            rc.SpeedM1(address1, self.speed)
            rc.SpeedM2(address2, self.speed)
        elif self.left == 2:
            rc.SpeedM1(address1, -self.speed)
            rc.SpeedM2(address2, -self.speed)
        else:
            rc.SpeedM2(address1, 0)
            rc.SpeedM1(address2, 0)

        if self.right == 1:
            rc.SpeedM2(address1, -self.speed)
            rc.SpeedM1(address2, -self.speed)
        elif self.right == 2:
            rc.SpeedM2(address1, self.speed)
            rc.SpeedM1(address2, self.speed)
        else:
            rc.SpeedM1(address1, 0)
            rc.SpeedM2(address2, 0)

    def on_up_arrow_press(self):
        self.left = 1
        self.updateMotors()

    def on_up_down_arrow_release(self):
        if self.left == 1:
            self.left = 0
        self.updateMotors()

    def on_L1_press(self):
        self.left = 2
        self.updateMotors()

    def on_L1_release(self):
        if self.left == 2:
            self.left = 0
        self.updateMotors()

    def on_triangle_press(self):
        self.right = 1
        self.updateMotors()

    def on_triangle_release(self):
        if self.right == 1:
            self.right = 0
        self.updateMotors()

    def on_R1_press(self):
        self.right = 2
        self.updateMotors()

    def on_R1_release(self):
        if self.right == 2:
            self.right = 0
        self.updateMotors()

    def on_x_press(self):
        print("X pressed - stopping all motors.")
        rc.SpeedM1(address1, 0)
        rc.SpeedM2(address1, 0)
        rc.SpeedM1(address2, 0)
        rc.SpeedM2(address2, 0)
        os._exit(0)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no_joystick', action='store_true', help='Use button control mode (L1/Up Arrow/R1/Triangle)')
    parser.add_argument('--max_speed', type=int, default=12000, help='Maximum speed for rover (default: 12000)')
    parser.add_argument('--dead_zone', type=int, default=2000, help='Dead zone for joystick values (default: 2000)')
    args = parser.parse_args()

    if args.max_speed < 0:
        raise ValueError(f"Invalid max_speed value: {args.max_speed}")
    if args.dead_zone < 0:
        raise ValueError(f"Invalid dead_zone value: {args.dead_zone}")

    try:
        while not os.path.exists("/dev/input/js0"):
            time.sleep(1)

        if args.no_joystick:
            controller = TriggerController(maxSpeed=args.max_speed, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            controller = JoystickController(maxSpeed=args.max_speed, deadZone=args.dead_zone, interface="/dev/input/js0", connecting_using_ds4drv=False)

        controller.listen()

    finally:
        print("Stopping all motors.")
        rc.SpeedM1(address1, 0)
        rc.SpeedM2(address1, 0)
        rc.SpeedM1(address2, 0)
        rc.SpeedM2(address2, 0)

if __name__ == '__main__':
    main()