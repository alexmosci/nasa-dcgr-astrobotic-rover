import threading
import socket
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
ADAPTIVE_SPEED_SCALE = 1.0
ADAPTIVE_SPEED_MIN = 0.4
ROVER_TRACK_WIDTH = 0.34

cameraX = 0.0
cameraY = 0.0
cameraHeading = 0.0
cameraLock = threading.Lock()
cameraReady = threading.Event()

rc = Roboclaw("/dev/ttyS0", 38400)
rc.Open()
address1 = 0x80
address2 = 0x81

r11 = r12 = r21 = r22 = 0.0
tX = tY = 0.0
dTheta = 0.0

def initCameraToPathTransform(pathStartX, pathStartY, pathStartYawRad):
    global r11, r12, r21, r22, tX, tY, dTheta

    with cameraLock:
        cX = float(cameraX)
        cY = float(cameraY)
        cYaw = float(cameraHeading)

    dTheta = pathStartYawRad - cYaw
    c, s = math.cos(dTheta), math.sin(dTheta)
    r11, r12, r21, r22 = c, -s, s, c

    rC0x = r11 * cX + r12 * cY
    rC0y = r21 * cX + r22 * cY
    tX = pathStartX - rC0x
    tY = pathStartY - rC0y

    print(f"Rotation offset: {math.degrees(dTheta):.2f} degrees")
    print(f"Translation X: {tX:.3f} m")
    print(f"Translation Y: {tY:.3f} m")

def getPoseInPath():
    with cameraLock:
        cX = float(cameraX)
        cY = float(cameraY)
        cYaw = float(cameraHeading)

    xP = r11 * cX + r12 * cY + tX
    yP = r21 * cX + r22 * cY + tY
    yawP = angleMod(cYaw + dTheta)
    return xP, yP, yawP

def startCameraServer(host, port):
    global cameraX, cameraY, cameraHeading

    try:
        serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serverSocket.bind((host, port))
        serverSocket.listen(1)
        print(f"Listening for camera data on port {port}...")

        conn, addr = serverSocket.accept()
        print(f"Connection established with {addr}")

        with conn:
            buffer = ''
            while True:
                data = conn.recv(1024).decode()
                if not data:
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        xStr, yStr, headingStr = line.strip().split(',')
                        with cameraLock:
                            cameraX = float(xStr)
                            cameraY = float(yStr)
                            # Edit based on input heading format (compass: 0°=N, CW+ vs math: 0=+X, CCW+)
                            cameraHeading = compassDegToMathRad(float(headingStr))
                            
                            if not cameraReady.is_set():
                                cameraReady.set()
                    except ValueError:
                        print(f"Invalid data received: {line}")
    except Exception as e:
        print(f"Server error: {e}")

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
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def update(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

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
    parser.add_argument('--time_step', type=float, default=0.5, help='Time step for control loop in seconds (default: 0.5)')
    parser.add_argument('--start', type=str, default=None, help="Starting coordinate in the form x,y (image frame). Defaults to start of path")
    parser.add_argument('--start_heading', type=float, default=None, help="Starting heading in image coordinates. Defaults to heading at start point")
    parser.add_argument('--port', type=int, default=1299, help='Port for position TCP server (default: 1299)')
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

    cameraThread = threading.Thread(target=startCameraServer, args=('', args.port), daemon=True)
    cameraThread.start()
    cameraReady.wait()

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

    initCameraToPathTransform(startX, startY, startHeadingRad)
    state = State(x=startX, y=startY, yaw=startHeadingRad)
    trajectory = TargetCourse(pathX, pathY)
    lastIndex = 0

    try:
        while lastIndex < len(pathX) - 1:
            x, y, yaw = getPoseInPath()
            state.update(x, y, yaw)
            lastIndex, curvature = purePursuitSteerControl(state, trajectory, lastIndex)
            leftCommand, rightCommand = computeWheelCommands(curvature, maxSpeed)

            rc.SpeedM2(address1, leftCommand)
            rc.SpeedM1(address2, leftCommand)
            rc.SpeedM1(address1, -rightCommand)
            rc.SpeedM2(address2, -rightCommand)
            time.sleep(timeStep)

    finally:
        print("Stopping all motors.")
        rc.SpeedM1(address1, 0)
        rc.SpeedM2(address1, 0)
        rc.SpeedM1(address2, 0)
        rc.SpeedM2(address2, 0)

if __name__ == '__main__':
    main()