import argparse
import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

def loadBinaryMap(binaryMapCsv):
    rawData = np.genfromtxt(binaryMapCsv, delimiter=',', dtype=str)
    binaryMatrix = np.array([[1 if val.strip().lower() == 'nan' else int(float(val)) for val in row] for row in rawData])
    if binaryMatrix.shape[0] > binaryMatrix.shape[1]:
        binaryMatrix = np.rot90(binaryMatrix, k=-1)
    return binaryMatrix

def loadCoveragePath(pathCsv):
    pathList = []
    with open(pathCsv, newline='') as csvFile:
        reader = csv.reader(csvFile)
        next(reader, None)
        for rowData in reader:
            if not rowData:
                continue
            try:
                x = float(rowData[1])
                y = float(rowData[2])
                heading = float(rowData[3])
            except (ValueError, IndexError):
                continue
            pathList.append((x, y, heading))
    return pathList

def saveCoveragePlan(coveragePathList, outputCsv):
    with open(outputCsv, 'w', newline='') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(['Order', 'X', 'Y', 'Heading'])
        for index, (x, y, heading) in enumerate(coveragePathList):
            writer.writerow([index, x, y, heading])

def calculateHeading(fromPoint, toPoint):
    if fromPoint is None or toPoint is None:
        return 0.0
    deltaX = toPoint[0] - fromPoint[0]
    deltaY = fromPoint[1] - toPoint[1]
    return math.degrees(math.atan2(deltaX, deltaY)) % 360

def plotCoverageMap(binaryMatrix, coveragePathList, outputImagePath):
    plt.figure(figsize=(10, 8))
    plt.imshow(binaryMatrix, cmap='gray_r', origin='upper')
    if coveragePathList:
        xs, ys, _ = zip(*coveragePathList)
        plt.plot(xs, ys, color='blue', linewidth=1)
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(outputImagePath, dpi=300)
    plt.close()

def removeDuplicates(pathList):
    uniquePoints = []
    for point in pathList:
        if uniquePoints and uniquePoints[-1][0] == point[0] and uniquePoints[-1][1] == point[1]:
            uniquePoints[-1] = point
        else:
            uniquePoints.append(point)
    return uniquePoints

def generateFillet(prevPoint, currPoint, nextPoint, radius, stepDegree):
    vectorPrev = np.array(currPoint) - np.array(prevPoint)
    vectorNext = np.array(nextPoint) - np.array(currPoint)
    normPrev = np.linalg.norm(vectorPrev)
    normNext = np.linalg.norm(vectorNext)
    if normPrev < 1e-6 or normNext < 1e-6:
        return []
    headingPrev = math.radians(calculateHeading(prevPoint, currPoint))
    headingNext = math.radians(calculateHeading(currPoint, nextPoint))
    angleDelta = ((headingNext - headingPrev + math.pi) % (2 * math.pi)) - math.pi
    if abs(angleDelta) < 1e-3:
        return []
    rawDistance = abs(radius * math.tan(angleDelta / 2))
    maxDistance = max(min(normPrev, normNext) * 0.5 - 1e-3, 0)
    distance = min(rawDistance, maxDistance)
    turnSign = 1.0 if angleDelta > 0 else -1.0
    unitVectorPrev = vectorPrev / normPrev
    entryPoint = np.array(currPoint) - unitVectorPrev * distance
    perpVector = np.array([-unitVectorPrev[1], unitVectorPrev[0]]) * turnSign
    centerPoint = entryPoint + perpVector * radius
    startAngle = math.atan2(entryPoint[1] - centerPoint[1], entryPoint[0] - centerPoint[0])
    endAngle = startAngle + angleDelta
    stepRadians = math.radians(stepDegree)
    numPoints = max(3, int(abs(endAngle - startAngle) / stepRadians))
    angleList = np.linspace(startAngle, endAngle, numPoints)
    arcPoints = []
    for idx, angle in enumerate(angleList):
        x = centerPoint[0] + radius * math.cos(angle)
        y = centerPoint[1] + radius * math.sin(angle)
        if idx == 0:
            heading = calculateHeading(prevPoint, (x, y))
        elif idx == len(angleList) - 1:
            heading = calculateHeading((x, y), nextPoint)
        else:
            x_prev = centerPoint[0] + radius * math.cos(angleList[idx - 1])
            y_prev = centerPoint[1] + radius * math.sin(angleList[idx - 1])
            x_next = centerPoint[0] + radius * math.cos(angleList[min(idx + 1, len(angleList)-1)])
            y_next = centerPoint[1] + radius * math.sin(angleList[min(idx + 1, len(angleList)-1)])
            heading = calculateHeading((x_prev, y_prev), (x_next, y_next))
        arcPoints.append((x, y, heading))
    return arcPoints

def extendArc(arcPoints, radius, stepDegree, numExtraPoints=20):
    # Need at least three points to estimate circle center
    if len(arcPoints) < 3:
        return arcPoints
    x1, y1, _ = arcPoints[-3]
    x2, y2, _ = arcPoints[-2]
    x3, y3, _ = arcPoints[-1]
    def circle_center(x1, y1, x2, y2, x3, y3):
        temp = x2**2 + y2**2
        bc = (x1**2 + y1**2 - temp) / 2.0
        cd = (temp - x3**2 - y3**2) / 2.0
        det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)
        if abs(det) < 1e-10:
            return None, None
        cx = (bc*(y2 - y3) - cd*(y1 - y2)) / det
        cy = ((x1 - x2)*cd - (x2 - x3)*bc) / det
        return cx, cy
    cx, cy = circle_center(x1, y1, x2, y2, x3, y3)
    if cx is None or cy is None:
        return arcPoints
    angle_last = math.atan2(y3 - cy, x3 - cx)
    cross = (x2-x1)*(y3-y2) - (y2-y1)*(x3-x2)
    sign = 1.0 if cross > 0 else -1.0
    stepRadians = math.radians(stepDegree) * sign
    for i in range(1, numExtraPoints + 1):
        angle = angle_last + i * stepRadians
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        heading = (math.degrees(angle + sign * math.pi/2)) % 360
        arcPoints.append((x, y, heading))
    return arcPoints

def adjustArcEntry(previousArc, currentArc, radius, stepDegree, maxAttempts=5, extraPointsPerAttempt=20):
    if not previousArc or not currentArc:
        return currentArc

    previousPoint = np.array(previousArc[-1])
    bestIndex = 0
    minError = float('inf')
    for attempt in range(maxAttempts):
        for i in range(1, len(currentArc) - 1):
            p1 = np.array(currentArc[i - 1][:2])
            p2 = np.array(currentArc[i + 1][:2])
            tangent = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            direction = math.atan2(currentArc[i][1] - previousPoint[1], currentArc[i][0] - previousPoint[0])
            angleError = abs(((tangent - direction + math.pi) % (2 * math.pi)) - math.pi)
            if angleError < minError:
                minError = angleError
                bestIndex = i
        if bestIndex <= 1 and attempt < maxAttempts - 1:
            revArc = list(reversed(currentArc))
            extended = extendArc(revArc, radius, stepDegree, numExtraPoints=extraPointsPerAttempt)
            currentArc = list(reversed(extended))
        else:
            break
    return currentArc[bestIndex:]

def adjustArcExit(currentArc, nextArc, radius, stepDegree, maxAttempts=5, extraPointsPerAttempt=20):
    if not currentArc or not nextArc:
        return currentArc

    nextPoint = np.array(nextArc[0][:2])
    bestIndex = len(currentArc) - 1
    minError = float('inf')
    for attempt in range(maxAttempts):
        for i in range(1, len(currentArc) - 1):
            p1 = np.array(currentArc[i - 1][:2])
            p2 = np.array(currentArc[i + 1][:2])
            tangent = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            direction = math.atan2(nextPoint[1] - currentArc[i][1], nextPoint[0] - currentArc[i][0])
            angleError = abs(((tangent - direction + math.pi) % (2 * math.pi)) - math.pi)
            if angleError < minError:
                minError = angleError
                bestIndex = i
        if bestIndex >= len(currentArc) - 2 and attempt < maxAttempts - 1:
            currentArc = extendArc(currentArc, radius, stepDegree, numExtraPoints=extraPointsPerAttempt)
        else:
            break
    return currentArc[:bestIndex + 1]

def smoothPathTurningRadius(pathWithHeadingList, turnRadius, stepDegree):
    pointList = [(x, y) for x, y, _ in pathWithHeadingList]
    if len(pointList) < 3:
        return [(float(x), float(y), 0.0) for x, y in pointList]

    arcsList = []
    for i in range(1, len(pointList) - 1):
        arcPoints = generateFillet(pointList[i - 1], pointList[i], pointList[i + 1], turnRadius, stepDegree)
        if arcPoints:
            arcsList.append(arcPoints)

    for i in range(1, len(arcsList)):
        arcsList[i] = adjustArcEntry(arcsList[i - 1], arcsList[i], turnRadius, stepDegree)
        arcsList[i - 1] = adjustArcExit(arcsList[i - 1], arcsList[i], turnRadius, stepDegree)

    smoothedSequence = [(*pointList[0], 0.0)]
    for arc in arcsList:
        smoothedSequence.extend(arc)
    smoothedSequence.append((*pointList[-1], smoothedSequence[-1][2]))

    return smoothedSequence

def main():
    parser = argparse.ArgumentParser(description="Generate a coverage plan respecting a given turning radius.")
    parser.add_argument("input_path", help="Input coverage path CSV file")
    parser.add_argument("output_path", help="Output coverage path CSV file")
    parser.add_argument("--binary", type=str, default=None, help="Binary map CSV file")
    parser.add_argument("--turning_radius", type=float, default=0.5, help="Turning radius of the robot (default: 0.5)")
    parser.add_argument("--step_degree", type=float, default=5.0, help="Angular step in degrees for arc sampling (default: 5)")
    parser.add_argument("--scale", type=str, default="1", help="Scale factor for real-world units. Can be a fraction (default: 1)")
    args = parser.parse_args()

    try:
        if '/' in args.scale:
            numerator, denominator = args.scale.split('/')
            scaleFactor = float(numerator) / float(denominator)
        else:
            scaleFactor = float(args.scale)
    except Exception:
        raise ValueError(f"Invalid scale value: {args.scale}")

    turnRadius = args.turning_radius / scaleFactor
    stepDegree = args.step_degree

    inputCsv = args.input_path
    outputCsv = args.output_path
    binaryCsv = args.binary if args.binary else inputCsv.rsplit('.csv', 1)[0] + '_binary.csv'
    if not os.path.exists(inputCsv):
        raise FileNotFoundError(f"Coverage path file not found: {inputCsv}")
    if not os.path.exists(binaryCsv):
        raise FileNotFoundError(f"Binary map file not found: {binaryCsv}")

    binaryMatrix = loadBinaryMap(binaryCsv)
    rawPathList = loadCoveragePath(inputCsv)
    smoothedPathList = smoothPathTurningRadius(rawPathList, turnRadius, stepDegree)
    uniquePathList = removeDuplicates(smoothedPathList)
    finalPathList = []
    for index, point in enumerate(uniquePathList):
        nextPoint = uniquePathList[index + 1] if index + 1 < len(uniquePathList) else None
        headingAngle = calculateHeading(point, nextPoint) if nextPoint else calculateHeading(uniquePathList[index - 1], point) if index > 0 else 0.0
        finalPathList.append((float(point[0]), float(point[1]), headingAngle))

    coveragePathList = finalPathList
    outputImagePath = outputCsv.rsplit('.csv', 1)[0] + '.png'
    plotCoverageMap(binaryMatrix, coveragePathList, outputImagePath)
    print(f"Coverage image saved to: {outputImagePath}")
    saveCoveragePlan(coveragePathList, outputCsv)
    print(f"Coverage path saved to: {outputCsv}")

if __name__ == "__main__":
    main()