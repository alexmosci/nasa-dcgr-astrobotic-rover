import math
import numpy as np
import csv
import argparse
import os
import time
import sys
sys.path.append("/home/astrobotic/Documents/SDK/MotorControllers/roboclaw/Libraries/Python/roboclaw_python")
from roboclaw_3 import Roboclaw

LOOKAHEAD_DISTANCE = 0.5
TURN_GAIN = 1.0
ADAPTIVE_SPEED_SCALE = 0.5
ADAPTIVE_SPEED_MIN = 0.2
ROVER_TRACK_WIDTH = 0.34

iioPath = "/sys/bus/iio/devices/iio:device0"
rc = Roboclaw("/dev/ttyS0", 38400)
rc.Open()
address1 = 0x80
address2 = 0x81

def rFloat(path):
    with open(path, "r") as f:
        return float(f.read().strip())

def initIMU():
    calib = {
        "axBias": rFloat(f"{iioPath}/in_accel_x_calibbias"),
        "ayBias": rFloat(f"{iioPath}/in_accel_y_calibbias"),
        "aScale": rFloat(f"{iioPath}/in_accel_scale"),
        "gzBias": rFloat(f"{iioPath}/in_anglvel_z_calibbias"),
        "gScale": rFloat(f"{iioPath}/in_anglvel_scale"),
    }
    return calib

def readIMU(calib):
    axRaw = rFloat(f"{iioPath}/in_accel_x_raw")
    ayRaw = rFloat(f"{iioPath}/in_accel_y_raw")
    gzRaw = rFloat(f"{iioPath}/in_anglvel_z_raw")

    accelX = (axRaw + calib["axBias"]) * calib["aScale"]
    accelY = (ayRaw + calib["ayBias"]) * calib["aScale"]
    yawRateCW = (gzRaw + calib["gzBias"]) * calib["gScale"]
    yawRate = -yawRateCW
    return accelX, accelY, yawRate

def angleMod(x):
    return (x + math.pi) % (2 * math.pi) - math.pi

def compassDegToMathRad(compassDeg):
    return angleMod(math.radians(90.0 - compassDeg))

def loadCoveragePath(csvPath):
    pathX, pathY, pathYawRad = [], [], []
    with open(csvPath, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['X'])
            y = float(row['Y'])
            headingCompassDeg = float(row['Heading'])
            pathX.append(x)
            pathY.append(-y)
            pathYawRad.append(compassDegToMathRad(headingCompassDeg))
    return pathX, pathY, pathYawRad

class State:
    def __init__(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, accelX, accelY, yawRate, dt):
        self.yaw = angleMod(self.yaw + yawRate * dt)
        accelMag = math.hypot(accelX, accelY)
        self.v += accelMag * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def calcDistance(self, px, py):
        dx = self.x - px
        dy = self.y - py
        return math.hypot(dx, dy)

class TargetCourse:
    def __init__(self, pathX, pathY):
        self.pathX = pathX
        self.pathY = pathY
        self.oldNearestIndex = None

    def searchTargetIndex(self, state):
        if self.oldNearestIndex is None:
            dx = [state.x - ix for ix in self.pathX]
            dy = [state.y - iy for iy in self.pathY]
            d = np.hypot(dx, dy)
            index = int(np.argmin(d))
            self.oldNearestIndex = index
        else:
            index = self.oldNearestIndex
            while index + 1 < len(self.pathX):
                distThis = state.calcDistance(self.pathX[index], self.pathY[index])
                distNext = state.calcDistance(self.pathX[index + 1], self.pathY[index + 1])
                if distThis < distNext:
                    break
                index += 1
            self.oldNearestIndex = index
        while LOOKAHEAD_DISTANCE > state.calcDistance(self.pathX[index], self.pathY[index]):
            if (index + 1) >= len(self.pathX):
                break
            index += 1
        return index

def purePursuitSteerControl(state, trajectory, lastIndex):
    index = trajectory.searchTargetIndex(state)
    if lastIndex >= index:
        index = lastIndex
    if index >= len(trajectory.pathX):
        index = len(trajectory.pathX) - 1
    targetX = trajectory.pathX[index]
    targetY = trajectory.pathY[index]
    alpha = math.atan2(targetY - state.y, targetX - state.x) - state.yaw
    alpha = angleMod(alpha)
    alpha = max(-math.pi/2, min(math.pi/2, alpha))
    curvature = 2.0 * math.sin(alpha) / LOOKAHEAD_DISTANCE
    return index, curvature

def computeWheelCommands(curvature, maxSpeed):
    turnEffect = (ROVER_TRACK_WIDTH / 2.0) * TURN_GAIN
    leftSpeed = 1.0 - (curvature * turnEffect)
    rightSpeed = 1.0 + (curvature * turnEffect)
    maxWheelSpeed = max(abs(leftSpeed), abs(rightSpeed), 1e-3)
    scale = maxSpeed / maxWheelSpeed
    leftSpeed *= scale
    rightSpeed *= scale
    speedFactor = min(1.0, ADAPTIVE_SPEED_SCALE / (abs(curvature) + 0.01))
    speedFactor = max(speedFactor, ADAPTIVE_SPEED_MIN)
    leftCommand = int(leftSpeed * speedFactor)
    rightCommand = int(rightSpeed * speedFactor)
    return leftCommand, rightCommand

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("coverage_path", type=str, help="Path to the coverage path CSV file")
    parser.add_argument('--max_speed', type=int, default=12000, help='Maximum speed for rover (default: 12000)')
    parser.add_argument('--time_step', type=float, default=0.05, help='Time step for control loop in seconds (default: 0.05)')
    parser.add_argument('--start', type=str, default=None, help="Starting coordinate in the form x,y (image frame). Defaults to start of path")
    parser.add_argument('--start_heading', type=float, default=None, help="Starting heading in image coordinates (compass degrees). Defaults to heading at start point")
    args = parser.parse_args()

    if not args.coverage_path.lower().endswith('.csv'):
        raise ValueError("Coverage path must have a .csv extension")
    if not os.path.exists(args.coverage_path):
        raise FileNotFoundError(f"Coverage path file not found: {args.coverage_path}")
    if args.max_speed < 0:
        raise ValueError(f"Invalid max_speed value: {args.max_speed}")
    if args.time_step <= 0:
        raise ValueError(f"Invalid time_step value: {args.time_step}")

    timeStep = args.time_step
    maxSpeed = args.max_speed

    pathX, pathY, pathYawRad = loadCoveragePath(args.coverage_path)
    if args.start is None:
            startX = pathX[0]
            startY = pathY[0]
    else:
        try:
            xStr, yStr = args.start.split(',')
            startX = float(xStr)
            startY = -float(yStr)
        except Exception:
            raise ValueError(f"Invalid format for --start. Expected x,y but got: {args.start}")

    if args.start_heading is None:
        startHeadingRad = pathYawRad[0]
    else:
        try:
            startHeadingRad = compassDegToMathRad(float(args.start_heading))
        except Exception:
            raise ValueError(f"Invalid format for --start_heading. Expected a number, but got: {args.start_heading}")

    calib = initIMU()
    state = State(x=startX, y=startY, yaw=startHeadingRad, v=0.0)
    trajectory = TargetCourse(pathX, pathY)
    lastIndex = 0

    try:
        while lastIndex < len(pathX) - 1:
            accelX, accelY, yawRate = readIMU(calib)
            lastIndex, curvature = purePursuitSteerControl(state, trajectory, lastIndex)

            leftCommand, rightCommand = computeWheelCommands(curvature, maxSpeed)
            rc.SpeedM2(address1, leftCommand)
            rc.SpeedM1(address2, leftCommand)
            rc.SpeedM1(address1, -rightCommand)
            rc.SpeedM2(address2, -rightCommand)

            state.update(accelX, accelY, yawRate, timeStep)
            time.sleep(timeStep)
    finally:
        print("Stopping all motors.")
        rc.SpeedM1(address1, 0)
        rc.SpeedM2(address1, 0)
        rc.SpeedM1(address2, 0)
        rc.SpeedM2(address2, 0)

if __name__ == '__main__':
    main()