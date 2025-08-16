import argparse
import os
import csv
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import math

def breadthFirstSearch(start, goal, binaryMap):
    height, width = binaryMap.shape
    queue = deque([start])
    cameFrom = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            break
        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if (0 <= int(nx) < width and 0 <= int(ny) < height 
                and binaryMap[int(ny), int(nx)] == 0):
                neighbor = (nx, ny)
                if neighbor not in cameFrom:
                    cameFrom[neighbor] = current
                    queue.append(neighbor)
    if goal not in cameFrom:
        return []
    curr = goal
    reversePath = []
    while curr is not None:
        reversePath.append(curr)
        curr = cameFrom[curr]
    return reversePath[::-1]

def calculateHeading(p1, p2):
    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]
    angle = math.degrees(math.atan2(dx, dy)) % 360
    return angle

def loadDecomposition(decompositionCsv):
    cells = {}
    with open(decompositionCsv, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            cellId = int(row['CellID'])
            vertexId = int(row['VertexID'])
            x = float(row['X'])
            y = float(row['Y'])
            if cellId not in cells:
                cells[cellId] = []
            cells[cellId].append((vertexId, x, y))
    return cells

def loadTraversalOrder(orderCsv):
    with open(orderCsv, newline='') as csvfile:
        reader = csv.reader(csvfile)
        row = next(reader)
        return [int(cellId.strip()) for cellId in row if cellId.strip()]

def loadBinaryMap(binaryMapCsv):
    raw = np.genfromtxt(binaryMapCsv, delimiter=',', dtype=str)
    binary = np.array([[1 if val.strip().lower() == 'nan' else int(float(val)) for val in row] for row in raw])

    if binary.shape[0] > binary.shape[1]:
        binary = np.rot90(binary, k=-1)
    return binary

def saveCoveragePlan(coveragePath, outputCsv):
    with open(outputCsv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Order', 'X', 'Y', 'Heading'])
        for i, (x, y, heading) in enumerate(coveragePath):
            writer.writerow([i, x, y, heading])

def plotCoverageMap(binaryMap, coveragePath, outputImagePath):
    plt.figure(figsize=(10, 8))
    plt.imshow(binaryMap, cmap='gray_r', origin='upper')

    xs, ys, _ = zip(*coveragePath) if coveragePath else ([], [], [])
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

def lawnMower(cellId, cellMap, coverageDiameter, startFromTop=True):
    verts = cellMap[cellId]
    cols = {}
    for _, x, y in verts:
        cols.setdefault(x, []).append(y)

    ordered, fromTop = [], startFromTop
    step = int(coverageDiameter)
    for i, x in enumerate(sorted(cols)):
        ys = sorted(set(cols[x]))
        if len(ys) < 2:
            ordered.append((x, ys[0]))
            fromTop = not fromTop
        else:
            if i % step == 0:
                pair = [(x, ys[0]), (x, ys[1])] if fromTop else [(x, ys[1]), (x, ys[0])]
                ordered.extend(pair)
                fromTop = not fromTop
            else:
                ordered.append((x, ys[0] if fromTop else ys[1]))
    return ordered

def generateCoveragePath(cellMap, cellOrder, mapBinary, coverageDiameter, startPoint):
    result = []
    current = startPoint
    scannedCells = set()

    if startPoint:
        firstCellId = next((cid for cid in cellOrder if cid in cellMap), None)
        if firstCellId:
            verts = cellMap[firstCellId]
            xs = [v[1] for v in verts]
            xMin = min(xs)
            left = [v for v in verts if v[1] == xMin]
            topLeft = min(left or verts, key=lambda v: v[2])
            entryPt = (topLeft[1], topLeft[2])
            if current != entryPt:
                result.extend(breadthFirstSearch(current, entryPt, mapBinary))
            current = entryPt
            result.append(current)

    for idx, cid in enumerate(cellOrder):
        verts = cellMap.get(cid)
        if not verts:
            continue

        if cid in scannedCells:
            nextCid = None
            for futureCid in cellOrder[idx+1:]:
                if futureCid not in scannedCells:
                    nextCid = futureCid
                    break
            if nextCid is not None:
                futureVerts = cellMap.get(nextCid)
                xsF = [v[1] for v in futureVerts]
                xMinF, xMaxF = min(xsF), max(xsF)
                leftF = [v for v in futureVerts if v[1] == xMinF]
                rightF = [v for v in futureVerts if v[1] == xMaxF]
                tL = min(leftF or futureVerts, key=lambda v: v[2])
                bL = max(leftF or futureVerts, key=lambda v: v[2])
                tR = min(rightF or futureVerts, key=lambda v: v[2])
                bR = max(rightF or futureVerts, key=lambda v: v[2])
                corners = [(tL[1], tL[2]), (bL[1], bL[2]), (tR[1], tR[2]), (bR[1], bR[2])]
                if current:
                    entryPt = min(corners, key=lambda p: math.hypot(current[0]-p[0], current[1]-p[1]))
                else:
                    entryPt = corners[0]
                if current and current != entryPt:
                    result.extend(breadthFirstSearch(current, entryPt, mapBinary))
                else:
                    result.append(entryPt)
                current = entryPt
            continue

        xs = [v[1] for v in verts]
        xMin, xMax = min(xs), max(xs)
        left = [v for v in verts if v[1] == xMin]
        right = [v for v in verts if v[1] == xMax]

        topLeft = min(left or verts, key=lambda v: v[2])
        bottomLeft = max(left or verts, key=lambda v: v[2])
        topRight = min(right or verts, key=lambda v: v[2])
        bottomRight = max(right or verts, key=lambda v: v[2])

        horizCollapse = (topLeft == topRight and bottomLeft == bottomRight)
        vertCollapse = (topLeft == bottomLeft and topRight == bottomRight)
        if horizCollapse or vertCollapse:
            pts = [(corner[1], corner[2]) for corner in (topLeft, bottomLeft, topRight, bottomRight)]
            uniquePts = []
            for pt in pts:
                if pt not in uniquePts:
                    uniquePts.append(pt)
            if len(uniquePts) > 1:
                a, b = min(
                    ((p, q) for i, p in enumerate(uniquePts) for q in uniquePts[i+1:]),
                    key=lambda pair: math.hypot(pair[0][0]-pair[1][0], pair[0][1]-pair[1][1])
                )
                if current:
                    entryPt, otherPt = sorted([a, b], key=lambda p: math.hypot(current[0]-p[0], current[1]-p[1]))
                else:
                    entryPt, otherPt = a, b
            else:
                entryPt = uniquePts[0]
                otherPt = uniquePts[0]
            if not current:
                result.append(entryPt)
            elif current != entryPt:
                result.extend(breadthFirstSearch(current, entryPt, mapBinary))
            if entryPt != otherPt:
                result.extend(breadthFirstSearch(entryPt, otherPt, mapBinary))
            current = otherPt
            scannedCells.add(cid)
            continue
        
        corners = [
            (topLeft[1], topLeft[2]),
            (bottomLeft[1], bottomLeft[2]),
            (topRight[1], topRight[2]),
            (bottomRight[1], bottomRight[2])
        ]
        if current:
            entryPt = min(corners, key=lambda p: math.hypot(current[0]-p[0], current[1]-p[1]))
        else:
            entryPt = corners[0]

        if not current:
            result.append(entryPt)
        elif current != entryPt:
            result.extend(breadthFirstSearch(current, entryPt, mapBinary))

        startTop = entryPt == (topLeft[1], topLeft[2]) or entryPt == (topRight[1], topRight[2])
        cellMap[cid] = rotateVertexList(
            verts, topLeft, bottomLeft, topRight, bottomRight,
            next(v for v in (topLeft, bottomLeft, topRight, bottomRight) if (v[1], v[2]) == entryPt)
        )
        path = lawnMower(cid, cellMap, coverageDiameter, startTop)
        if path:
            if result[-1] != path[0]:
                result.extend(breadthFirstSearch(result[-1], path[0], mapBinary))
            result.extend(path)
            current = result[-1]

        scannedCells.add(cid)

    result = removeDuplicates(result)
    finalPath = []
    for i, pt in enumerate(result):
        nextPt = result[i+1] if i+1 < len(result) else None
        heading = calculateHeading(pt, nextPt) if nextPt else (finalPath[-1][2] if finalPath else 0)
        finalPath.append((float(pt[0]), float(pt[1]), heading))

    return finalPath

def rotateVertexList(vertexList, topLeft, bottomLeft, topRight, bottomRight, bestCorner):
    if bestCorner == topLeft:
        i = vertexList.index(topLeft)
        return vertexList[i:] + vertexList[:i]
    if bestCorner == bottomLeft:
        return list(reversed(vertexList))
    if bestCorner == topRight:
        i = vertexList.index(bestCorner)
        return vertexList[i::-1] + vertexList[:i:-1]
    if bestCorner == bottomRight:
        i = vertexList.index(bestCorner)
        return vertexList[i:] + vertexList[:i]
    return list(vertexList)

def main():
    parser = argparse.ArgumentParser(description="Generate a coverage plan from a decomposition and traversal order.")
    parser.add_argument("decomposition_csv", help="Input decomposition CSV file")
    parser.add_argument("coverage_path_csv", help="Output coverage path CSV file")
    parser.add_argument("--order", type=str, default=None, help="Traversal order CSV file (default: based on decomposition CSV file)")
    parser.add_argument("--binary", type=str, default=None, help="Binary map CSV file (default: based on decomposition CSV file)")
    parser.add_argument("--scale", type=str, default="1", help="Scale factor for real-world units. Can be a fraction (default: 1)")
    parser.add_argument("--coverage_diameter", type=float, default=1.0, help="Diameter of coverage per pass (default: 1.0)")
    parser.add_argument("--start", type=str, default=None, help="Optional start coordinate in the form x,y")
    args = parser.parse_args()

    if not args.decomposition_csv.lower().endswith('.csv'):
        raise ValueError("Decomposition file must have a .csv extension")
    if not os.path.exists(args.decomposition_csv):
        raise FileNotFoundError(f"Decomposition file not found: {args.decomposition_csv}")

    orderCsv = args.order if args.order else args.decomposition_csv.rsplit(".csv", 1)[0] + "_order.csv"
    if not os.path.exists(orderCsv):
        raise FileNotFoundError(f"Traversal order file not found: {orderCsv}")

    binaryCsv = args.binary if args.binary else args.decomposition_csv.rsplit(".csv", 1)[0] + "_binary.csv"
    if not os.path.exists(binaryCsv):
        raise FileNotFoundError(f"Binary map file not found: {binaryCsv}")

    try:
        if '/' in args.scale:
            numerator, denominator = args.scale.split('/')
            scale = float(numerator) / float(denominator)
        else:
            scale = float(args.scale)
    except Exception:
        raise ValueError(f"Invalid scale value: {args.scale}")

    coverageDiameter = args.coverage_diameter / scale
    if int(coverageDiameter) <= 0:
        raise ValueError("Coverage diameter is too small relative to the resolution of the binary map")

    startPoint = None
    if args.start:
        try:
            x_str, y_str = args.start.split(',')
            startPoint = (float(x_str), float(y_str))
        except Exception:
            raise ValueError(f"Invalid format for --start. Expected x,y but got: {args.start}")

    cells = loadDecomposition(args.decomposition_csv)
    traversalOrder = loadTraversalOrder(orderCsv)
    binaryMap = loadBinaryMap(binaryCsv)

    binaryOutput = args.coverage_path_csv.rsplit(".csv", 1)[0] + "_Binary.csv"
    np.savetxt(binaryOutput, binaryMap, fmt="%d", delimiter=",")
    print(f"Binary map saved to: {binaryOutput}")

    coveragePath = generateCoveragePath(cells, traversalOrder, binaryMap, coverageDiameter, startPoint)
    outputCsv = args.coverage_path_csv
    outputPng = args.coverage_path_csv.rsplit(".csv", 1)[0] + ".png"

    plotCoverageMap(binaryMap, coveragePath, outputPng)
    print(f"Coverage image saved to: {outputPng}")
    saveCoveragePlan(coveragePath, outputCsv)
    print(f"Coverage path saved to: {outputCsv}")

if __name__ == "__main__":
    main()