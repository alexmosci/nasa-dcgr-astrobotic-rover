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
    with open(pathCsv, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            try:
                x = float(row[1])
                y = float(row[2])
                h = float(row[3])
            except (ValueError, IndexError):
                continue
            path.append((x, y, h))
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

def removeDuplicates(path):
    unique = []
    for point in path:
        if unique and unique[-1][0] == point[0] and unique[-1][1] == point[1]:
            unique[-1] = point
        else:
            unique.append(point)
    return unique

def remove180Turns(pathWithHeading):
    smoothed = []
    i = 0
    while i < len(pathWithHeading) - 1:
        p0 = pathWithHeading[i]
        p1 = pathWithHeading[i + 1]
        smoothed.append((p0[0], p0[1], p0[2]))
        heading_diff = abs(p1[2] - p0[2]) % 360
        if 179 <= heading_diff <= 181 and (i + 2 < len(pathWithHeading)):
            heading = p0[2] % 360
            if heading < 45 or heading >= 315:
                smoothed.extend([
                    (p1[0] + 0.5, p1[1] + 0.5, heading),
                    (p1[0], p1[1], heading),
                    (p1[0] - 0.5, p1[1] + 0.5, heading)
                ])
            elif 135 <= heading < 225:
                smoothed.extend([
                    (p1[0] - 0.5, p1[1] - 0.5, heading),
                    (p1[0], p1[1], heading),
                    (p1[0] + 0.5, p1[1] - 0.5, heading)
                ])
            elif 225 <= heading < 315:
                smoothed.extend([
                    (p1[0] + 0.5, p1[1] - 0.5, heading),
                    (p1[0], p1[1], heading),
                    (p1[0] + 0.5, p1[1] + 0.5, heading)
                ])
            else:
                smoothed.extend([
                    (p1[0] - 0.5, p1[1] + 0.5, heading),
                    (p1[0], p1[1], heading),
                    (p1[0] - 0.5, p1[1] - 0.5, heading)
                ])
            i += 2
        else:
            i += 1
    if pathWithHeading:
        smoothed.append(pathWithHeading[-1])
    smoothed = removeDuplicates(smoothed)
    final = []
    for idx, point in enumerate(smoothed):
        if idx + 1 < len(smoothed):
            h = calculateHeading(point, smoothed[idx + 1])
        else:
            h = smoothed[-1][2]
        final.append((point[0], point[1], h))
    return final

def chaikinCurve(path):
    newPoints = []
    for i in range(len(path) - 1):
        p0 = path[i]
        p1 = path[i + 1]
        q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
        r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
        newPoints.extend([q, r])
    return newPoints

def smoothPath(pathWithHeading, iterations):
    points = [(x, y) for x, y, _ in pathWithHeading]
    for _ in range(iterations):
        points = chaikinCurve(points)
    smoothed = []
    for i in range(len(points)):
        x, y = points[i]
        if i + 1 < len(points):
            h = calculateHeading(points[i], points[i + 1])
        else:
            h = 0.0
        smoothed.append((x, y, h))
    return smoothed

def main():
    parser = argparse.ArgumentParser(description="Generate a smooth coverage plan from a binary map and coverage path.")
    parser.add_argument("input_path", help="Input coverage path CSV file")
    parser.add_argument("output_path", help="Output coverage path CSV file")
    parser.add_argument("--binary", type=str, default=None, help="Binary map CSV file (default: based on input CSV)")
    args = parser.parse_args()

    inputCsv = args.input_path
    outputCsv = args.output_path
    binaryCsv = args.binary if args.binary else inputCsv.rsplit(".csv", 1)[0] + "_binary.csv"

    if not os.path.exists(inputCsv):
        raise FileNotFoundError(f"Coverage path file not found: {inputCsv}")
    if not os.path.exists(binaryCsv):
        raise FileNotFoundError(f"Binary map file not found: {binaryCsv}")

    binaryMap = loadBinaryMap(binaryCsv)
    path = loadCoveragePath(inputCsv)
    path = remove180Turns(path)
    path = smoothPath(path, 3)

    outputPng = outputCsv.rsplit(".csv", 1)[0] + ".png"
    plotCoverageMap(binaryMap, path, outputPng)
    print(f"Coverage image saved to: {outputPng}")
    saveCoveragePlan(path, outputCsv)
    print(f"Coverage path saved to: {outputCsv}")

if __name__ == "__main__":
    main()