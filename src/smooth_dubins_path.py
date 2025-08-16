import argparse
import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

def angleMod(x, zeroTo2Pi=False):
    a = np.arctan2(np.sin(x), np.cos(x))
    if zeroTo2Pi and a < 0:
        a += 2 * np.pi
    return a

def rotMat2d(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def pathSmoothingPlanner(startX, startY, startAngle, goalX, goalY, goalAngle, curvature, stepSize, segmentPlanners, goalThreshold):
    if segmentPlanners is None:
        return None, None, None, None, None

    localRot = rotMat2d(startAngle)
    localGoalXY = np.stack([goalX - startX, goalY - startY]).T @ localRot
    localGoalX = localGoalXY[0]
    localGoalY = localGoalXY[1]
    localGoalAngle = goalAngle - startAngle

    pathX, pathY, pathAngles, modes, lengths = coveragePathPlanningFromOrigin(
        localGoalX, localGoalY, localGoalAngle, curvature, stepSize, segmentPlanners.values(), goalThreshold
    )

    rot = rotMat2d(-startAngle)
    convertedXY = np.stack([pathX, pathY]).T @ rot
    xList = convertedXY[:, 0] + startX
    yList = convertedXY[:, 1] + startY
    angleList = angleMod(np.array(pathAngles) + startAngle)

    return xList, yList, angleList, modes, lengths

def mod2pi(theta):
    return angleMod(theta, zeroTo2Pi=True)

def calcTrigFuncs(alpha, beta):
    sinA = math.sin(alpha)
    sinB = math.sin(beta)
    cosA = math.cos(alpha)
    cosB = math.cos(beta)
    cosAB = math.cos(alpha - beta)
    return sinA, sinB, cosA, cosB, cosAB

def leftStraightLeft(alpha, beta, d):
    sinA, sinB, cosA, cosB, cosAB = calcTrigFuncs(alpha, beta)
    mode = ["L", "S", "L"]
    pSquared = 2 + d ** 2 - (2 * cosAB) + (2 * d * (sinA - sinB))
    if pSquared < 0:
        return None, None, None, mode
    tmp = math.atan2((cosB - cosA), d + sinA - sinB)
    d1 = mod2pi(-alpha + tmp)
    d2 = math.sqrt(pSquared)
    d3 = mod2pi(beta - tmp)
    return d1, d2, d3, mode

def rightStraightRight(alpha, beta, d):
    sinA, sinB, cosA, cosB, cosAB = calcTrigFuncs(alpha, beta)
    mode = ["R", "S", "R"]
    pSquared = 2 + d ** 2 - (2 * cosAB) + (2 * d * (sinB - sinA))
    if pSquared < 0:
        return None, None, None, mode
    tmp = math.atan2((cosA - cosB), d - sinA + sinB)
    d1 = mod2pi(alpha - tmp)
    d2 = math.sqrt(pSquared)
    d3 = mod2pi(-beta + tmp)
    return d1, d2, d3, mode

def leftStraightRight(alpha, beta, d):
    sinA, sinB, cosA, cosB, cosAB = calcTrigFuncs(alpha, beta)
    pSquared = -2 + d ** 2 + (2 * cosAB) + (2 * d * (sinA + sinB))
    mode = ["L", "S", "R"]
    if pSquared < 0:
        return None, None, None, mode
    d1 = math.sqrt(pSquared)
    tmp = math.atan2((-cosA - cosB), (d + sinA + sinB)) - math.atan2(-2.0, d1)
    d2 = mod2pi(-alpha + tmp)
    d3 = mod2pi(-mod2pi(beta) + tmp)
    return d2, d1, d3, mode

def rightStraightLeft(alpha, beta, d):
    sinA, sinB, cosA, cosB, cosAB = calcTrigFuncs(alpha, beta)
    pSquared = d ** 2 - 2 + (2 * cosAB) - (2 * d * (sinA + sinB))
    mode = ["R", "S", "L"]
    if pSquared < 0:
        return None, None, None, mode
    d1 = math.sqrt(pSquared)
    tmp = math.atan2((cosA + cosB), (d - sinA - sinB)) - math.atan2(2.0, d1)
    d2 = mod2pi(alpha - tmp)
    d3 = mod2pi(beta - tmp)
    return d2, d1, d3, mode

def rightLeftRight(alpha, beta, d):
    sinA, sinB, cosA, cosB, cosAB = calcTrigFuncs(alpha, beta)
    mode = ["R", "L", "R"]
    tmp = (6.0 - d ** 2 + 2.0 * cosAB + 2.0 * d * (sinA - sinB)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = mod2pi(2 * math.pi - math.acos(tmp))
    d1 = mod2pi(alpha - math.atan2(cosA - cosB, d - sinA + sinB) + d2 / 2.0)
    d3 = mod2pi(alpha - beta - d1 + d2)
    return d1, d2, d3, mode

def leftRightLeft(alpha, beta, d):
    sinA, sinB, cosA, cosB, cosAB = calcTrigFuncs(alpha, beta)
    mode = ["L", "R", "L"]
    tmp = (6.0 - d ** 2 + 2.0 * cosAB + 2.0 * d * (- sinA + sinB)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = mod2pi(2 * math.pi - math.acos(tmp))
    d1 = mod2pi(-alpha - math.atan2(cosA - cosB, d + sinA - sinB) + d2 / 2.0)
    d3 = mod2pi(mod2pi(beta) - alpha - d1 + mod2pi(d2))
    return d1, d2, d3, mode

def coveragePathPlanningFromOrigin(endX, endY, endAngle, curvature, stepSize, planningFuncs, goalThreshold):
    dx = endX
    dy = endY
    d = math.hypot(dx, dy) * curvature
    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(-theta)
    beta = mod2pi(endAngle - theta)
    bestCost = float("inf")
    bestD1, bestD2, bestD3, bestMode = None, None, None, None
    for planner in planningFuncs:
        d1, d2, d3, mode = planner(alpha, beta, d)
        if d1 is None:
            continue
        cost = (abs(d1) + abs(d2) + abs(d3))
        if bestCost > cost:
            bestD1, bestD2, bestD3, bestMode, bestCost = d1, d2, d3, mode, cost
    lengths = [bestD1, bestD2, bestD3]
    xList, yList, angleList = generateLocalCourse(lengths, bestMode, curvature, stepSize, endX, endY, goalThreshold)
    lengths = [length / curvature for length in lengths]
    return xList, yList, angleList, bestMode, lengths

def interpolateSegment(length, mode, maxCurvature, originX, originY, originAngle, pathX, pathY, pathAngle):
    if mode == "S":
        pathX.append(originX + length / maxCurvature * math.cos(originAngle))
        pathY.append(originY + length / maxCurvature * math.sin(originAngle))
        pathAngle.append(originAngle)
    else:
        ldx = math.sin(length) / maxCurvature
        ldy = 0.0
        if mode == "L":
            ldy = (1.0 - math.cos(length)) / maxCurvature
        elif mode == "R":
            ldy = (1.0 - math.cos(length)) / -maxCurvature
        gdx = math.cos(-originAngle) * ldx + math.sin(-originAngle) * ldy
        gdy = -math.sin(-originAngle) * ldx + math.cos(-originAngle) * ldy
        pathX.append(originX + gdx)
        pathY.append(originY + gdy)
        if mode == "L":
            pathAngle.append(originAngle + length)
        elif mode == "R":
            pathAngle.append(originAngle - length)
    return pathX, pathY, pathAngle

def generateLocalCourse(lengths, modes, maxCurvature, stepSize, goalX, goalY, goalThreshold):
    px, py, pang = [0.0], [0.0], [0.0]
    for (mode, length) in zip(modes, lengths):
        if length == 0.0:
            continue
        originX, originY, originAngle = px[-1], py[-1], pang[-1]
        currentLength = stepSize
        while abs(currentLength + stepSize) <= abs(length):
            px, py, pang = interpolateSegment(currentLength, mode, maxCurvature, originX, originY, originAngle, px, py, pang)
            currentLength += stepSize

            if goalX is not None and goalThreshold is not None:
                if math.hypot(px[-1] - goalX, py[-1] - goalY) <= goalThreshold:
                    return np.array(px), np.array(py), np.array(pang)

        px, py, pang = interpolateSegment(length, mode, maxCurvature, originX, originY, originAngle, px, py, pang)

        if goalX is not None and goalThreshold is not None:
            if math.hypot(px[-1] - goalX, py[-1] - goalY) <= goalThreshold:
                return np.array(px), np.array(py), np.array(pang)

    return np.array(px), np.array(py), np.array(pang)

def loadBinaryMap(binaryMapCsv):
    raw = np.genfromtxt(binaryMapCsv, delimiter=',', dtype=str)
    binary = np.array([[1 if val.strip().lower() == 'nan' else int(float(val)) for val in row] for row in raw])
    if binary.shape[0] > binary.shape[1]:
        binary = np.rot90(binary, k=-1)
    return binary

def loadCoveragePathNoHeading(pathCsv):
    path = []
    lastHeading = None
    lastPoint = None

    with open(pathCsv, newline='') as csvFile:
        reader = csv.reader(csvFile)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            try:
                x = float(row[1])
                y = float(row[2])
                heading = float(row[3])
            except (ValueError, IndexError):
                continue

            point = (x, y)
            lastPoint = point

            if lastHeading is None or abs(heading - lastHeading) > 1e-2:
                path.append(point)
                lastHeading = heading

    if lastPoint and (not path or path[-1] != lastPoint):
        path.append(lastPoint)

    return path

def saveCoveragePlan(coveragePath, outputCsv):
    with open(outputCsv, 'w', newline='') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(['Order', 'X', 'Y', 'Heading'])
        for i, (x, y, heading) in enumerate(coveragePath):
            writer.writerow([i, x, y, heading])

def plotCoverageMap(binaryMap, coveragePath, outputImagePath):
    plt.figure(figsize=(10, 8))
    plt.imshow(binaryMap, cmap='gray_r', origin='upper')
    if coveragePath:
        xs, ys, _ = zip(*coveragePath)
        plt.plot(xs, ys, color='blue', linewidth=1)
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(outputImagePath, dpi=300)
    plt.close()

def calculateHeading(p1, p2):
    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]
    angle = math.degrees(math.atan2(dx, dy)) % 360
    return angle

def pathTangents(pathXY):
    headings = []
    for i in range(len(pathXY)):
        if i == 0:
            dx = pathXY[i+1][0] - pathXY[i][0]
            dy = pathXY[i+1][1] - pathXY[i][1]
        elif i == len(pathXY)-1:
            dx = pathXY[i][0] - pathXY[i-1][0]
            dy = pathXY[i][1] - pathXY[i-1][1]
        else:
            dx = pathXY[i+1][0] - pathXY[i-1][0]
            dy = pathXY[i+1][1] - pathXY[i-1][1]
        headings.append(math.atan2(dy, dx))
    return headings

def smoothCoveragePath(pathXY, turningRadius, stepSize, segmentPlanners, goalThreshold):
    if len(pathXY) < 2:
        return [(x, y) for x, y in pathXY]

    headings = pathTangents(pathXY)
    smoothedPath = []
    maxCurvature = 1.0 / turningRadius

    xCurr, yCurr = pathXY[0]
    hCurr = headings[0]
    smoothedPath.append((xCurr, yCurr))

    for i in range(1, len(pathXY)):
        xNext, yNext = pathXY[i]
        hNext = headings[i]

        thresholdThisSegment = 0.0 if i == len(pathXY) - 1 else goalThreshold

        xList, yList, angleList, _, _ = pathSmoothingPlanner(
            xCurr, yCurr, hCurr, xNext, yNext, hNext, maxCurvature, stepSize, segmentPlanners, thresholdThisSegment
        )

        if xList is None:
            continue

        for xx, yy in zip(xList[1:], yList[1:]):
            smoothedPath.append((xx, yy))

        xCurr = xList[-1]
        yCurr = yList[-1]
        hCurr = angleList[-1]

    return smoothedPath

def removeDuplicates(path, turningRadius, stepSize):
    if not path:
        return []

    threshold = (turningRadius * stepSize) / 2.0
    filtered = [path[0]]

    for i in range(1, len(path)):
        x0, y0 = filtered[-1]
        x1, y1 = path[i]
        if abs(x1 - x0) >= threshold or abs(y1 - y0) >= threshold:
            filtered.append(path[i])

    return filtered

def addHeadingsToPath(cleaned):
    finalPath = []
    for i, pt in enumerate(cleaned):
        nextPt = cleaned[i+1] if i+1 < len(cleaned) else None
        if nextPt:
            heading = calculateHeading(pt, nextPt)
        else:
            heading = finalPath[-1][2] if finalPath else 0.0
        finalPath.append((float(pt[0]), float(pt[1]), heading))
    return finalPath

def main():
    parser = argparse.ArgumentParser(description="Smooth an existing coverage path by applying turn constraints.")
    parser.add_argument("input_path", help="Input coverage path CSV file")
    parser.add_argument("output_path", help="Output coverage path CSV file")
    parser.add_argument("--binary", type=str, default=None, help="Binary map CSV file")
    parser.add_argument("--turning_radius", type=float, default=0.5, help="Turning radius for path smoothing (default: 0.5)")
    parser.add_argument("--step_size", type=float, default=0.1, help="Smoothing step size (default: 0.1)")
    parser.add_argument("--threshold", type=float, default=1.0, help="Tolerance for reaching waypoints during smoothing (default: 1.0)")
    parser.add_argument("--scale", type=str, default="1", help="Scale factor for real-world units. Can be a fraction (default: 1)")
    args = parser.parse_args()

    try:
        if '/' in args.scale:
            numerator, denominator = args.scale.split('/')
            scale = float(numerator) / float(denominator)
        else:
            scale = float(args.scale)
        if scale <= 0:
            raise Exception()
    except Exception:
        raise ValueError(f"Invalid scale value: {args.scale}")

    try:
        turningRadius = args.turning_radius / scale
        if turningRadius <= 0:
            raise Exception()
    except Exception:
        raise ValueError(f"Invalid turning radius value (must be > 0 after scaling): {args.turning_radius}")

    try:
        stepSize = args.step_size
        if stepSize <= 0:
            raise Exception()
    except Exception:
        raise ValueError(f"Invalid step size value (must be > 0): {args.step_size}")
    
    try:
        threshold = args.threshold
        if threshold < 0:
            raise Exception()
    except Exception:
        raise ValueError(f"Invalid threshold value (must be >= 0): {args.threshold}")

    inputCsv = args.input_path
    outputCsv = args.output_path
    binaryCsv = args.binary if args.binary else inputCsv.rsplit(".csv", 1)[0] + "_binary.csv"

    if not os.path.exists(inputCsv):
        raise FileNotFoundError(f"Coverage path file not found: {inputCsv}")
    if not os.path.exists(binaryCsv):
        raise FileNotFoundError(f"Binary map file not found: {binaryCsv}")

    segmentPlanners = {
        "LSL": leftStraightLeft,
        "RSR": rightStraightRight,
        "LSR": leftStraightRight,
        "RSL": rightStraightLeft,
        "RLR": rightLeftRight,
        "LRL": leftRightLeft,
    }

    binaryMap = loadBinaryMap(binaryCsv)
    rawPath = loadCoveragePathNoHeading(inputCsv)
    smoothedPath = smoothCoveragePath(rawPath, turningRadius, stepSize, segmentPlanners, threshold)
    smoothedPath = removeDuplicates(smoothedPath, turningRadius, stepSize)
    finalPath = addHeadingsToPath(smoothedPath)

    outputPng = outputCsv.rsplit(".csv", 1)[0] + ".png"
    plotCoverageMap(binaryMap, finalPath, outputPng)
    print(f"Coverage image saved to: {outputPng}")
    saveCoveragePlan(finalPath, outputCsv)
    print(f"Coverage path saved to: {outputCsv}")

if __name__ == "__main__":
    main()