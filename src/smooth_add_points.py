import argparse
import os
import csv
import matplotlib.pyplot as plt
import numpy as np
import math

def loadBinaryMap(binaryMapCsv):
    raw = np.genfromtxt(binaryMapCsv, delimiter=',', dtype=str)
    binary = np.array([[1 if val.strip().lower() == 'nan' else int(float(val)) for val in row] for row in raw])
    if binary.shape[0] > binary.shape[1]:
        binary = np.rot90(binary, k=-1)
    return binary

def loadCoveragePath(pathCsv):
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
    with open(outputCsv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Order', 'X', 'Y', 'Heading'])
        for i, (x, y, heading) in enumerate(coveragePath):
            writer.writerow([i, x, y, heading])

def calculateHeading(p1, p2):
    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]
    angle = math.degrees(math.atan2(dx, dy)) % 360
    return angle

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

def addPoints(filteredPath, spacing):
    if len(filteredPath) < 2:
        return [(x, y, 0.0) for x, y in filteredPath]

    newPath = []
    for i in range(len(filteredPath) - 1):
        x1, y1 = filteredPath[i]
        x2, y2 = filteredPath[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        steps = max(int(dist / spacing), 1)
        for j in range(steps):
            t = j / steps
            x = x1 + t * dx
            y = y1 + t * dy
            newPath.append((x, y, 0.0))
    newPath.append((x2, y2, 0.0))

    finalPath = []
    for i in range(len(newPath)):
        x, y, _ = newPath[i]
        if i + 1 < len(newPath):
            h = calculateHeading(newPath[i], newPath[i + 1])
        else:
            h = newPath[-1][2]
        finalPath.append((x, y, h))

    return finalPath

def main():
    parser = argparse.ArgumentParser(description="Generate a uniformly sampled coverage plan from a binary map and coverage path.")
    parser.add_argument("input_path", help="Input coverage path CSV file")
    parser.add_argument("output_path", help="Output coverage path CSV file")
    parser.add_argument("--binary", type=str, default=None, help="Binary map CSV file (default: based on input CSV)")
    parser.add_argument("--spacing", type=float, default=0.1, help="Spacing between interpolated points (default: 0.1)")
    parser.add_argument("--scale", type=str, default="1", help="Scale factor for real-world units. Can be a fraction (default: 1)")
    args = parser.parse_args()

    try:
        spacing = args.spacing
        if spacing <= 0:
            raise Exception()
    except Exception:
        raise ValueError(f"Invalid step size value (must be > 0): {args.spacing}")

    try:
        if '/' in args.scale:
            numerator, denominator = args.scale.split('/')
            scaleFactor = float(numerator) / float(denominator)
        else:
            scaleFactor = float(args.scale)
        if scaleFactor <= 0:
            raise Exception()
    except Exception:
        raise ValueError(f"Invalid scale value: {args.scale}")

    spacing = spacing / scaleFactor
    inputCsv = args.input_path
    outputCsv = args.output_path
    binaryCsv = args.binary if args.binary else inputCsv.rsplit(".csv", 1)[0] + "_binary.csv"

    if not os.path.exists(inputCsv):
        raise FileNotFoundError(f"Coverage path file not found: {inputCsv}")
    if not os.path.exists(binaryCsv):
        raise FileNotFoundError(f"Binary map file not found: {binaryCsv}")

    binaryMap = loadBinaryMap(binaryCsv)
    path = loadCoveragePath(inputCsv)
    path = addPoints(path, spacing)

    outputPng = outputCsv.rsplit(".csv", 1)[0] + ".png"
    plotCoverageMap(binaryMap, path, outputPng)
    print(f"Coverage image saved to: {outputPng}")
    saveCoveragePlan(path, outputCsv)
    print(f"Coverage path saved to: {outputCsv}")

if __name__ == "__main__":
    main()