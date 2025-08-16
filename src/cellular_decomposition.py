import numpy as np
import argparse
import csv
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from collections import defaultdict
import networkx as nx

def loadBinaryMap(inputCsv):
    raw = np.genfromtxt(inputCsv, delimiter=',', dtype=str)
    binary = np.array([[1 if val.strip().lower() == 'nan' else int(float(val)) for val in row] for row in raw])

    if binary.shape[0] > binary.shape[1]:
        binary = np.rot90(binary, k=-1)
        print("Input map rotated 90 degrees clockwise")
    else:
        print("Input map orientation unchanged")

    return binary

def countTransitions(column):
    transitions = 0
    for i in range(1, len(column)):
        if column[i] != column[i-1]:
            transitions += 1
    return transitions

def getFreeSegments(column):
    segments = []
    inSegment = False
    start = 0
    for i, val in enumerate(column):
        if val == 0 and not inSegment:
            start = i
            inSegment = True
        elif val == 1 and inSegment:
            segments.append((start, i))
            inSegment = False
    if inSegment:
        segments.append((start, len(column)))
    return segments

def extractCellsUsingConnectivity(binaryMap):
    height, width = binaryMap.shape
    cells = {}
    cellIdCounter = 0
    prevSegments = getFreeSegments(binaryMap[:, 0])
    activeCells = []

    for seg in prevSegments:
        top = [(0, seg[0])]
        bottom = [(0, seg[1])]
        cells[cellIdCounter] = (top, bottom)
        activeCells.append((seg, cellIdCounter))
        cellIdCounter += 1

    for x in range(1, width):
        currColumn = binaryMap[:, x]
        currSegments = getFreeSegments(currColumn)

        currToPrev = {i: [] for i in range(len(currSegments))}
        prevToCurr = {i: [] for i in range(len(prevSegments))}

        for i, currSeg in enumerate(currSegments):
            for j, (prevSeg, _) in enumerate(activeCells):
                if max(prevSeg[0], currSeg[0]) < min(prevSeg[1], currSeg[1]):
                    currToPrev[i].append(j)
                    prevToCurr[j].append(i)

        newActive = []
        usedPrev = set()

        for i, seg in enumerate(currSegments):
            localSplit = len(currToPrev[i]) > 1
            localMerge = any(len(prevToCurr[j]) > 1 for j in currToPrev[i])
            localCriticalEvent = localSplit or localMerge

            matched = False
            for j in currToPrev[i]:
                if not localCriticalEvent and j not in usedPrev:
                    prevSeg, cid = activeCells[j]
                    top, bottom = cells[cid]
                    top.append((x, seg[0]))
                    bottom.append((x, seg[1]))
                    cells[cid] = (top, bottom)
                    newActive.append((seg, cid))
                    usedPrev.add(j)
                    matched = True
                    break

            if not matched:
                top = [(x - 1, seg[0]), (x, seg[0])]
                bottom = [(x - 1, seg[1]), (x, seg[1])]
                cells[cellIdCounter] = (top, bottom)
                newActive.append((seg, cellIdCounter))
                cellIdCounter += 1

        activeCells = newActive
        prevSegments = currSegments

    for seg, cid in activeCells:
        top, bottom = cells[cid]
        top.append((width - 1, seg[0]))
        bottom.append((width - 1, seg[1]))
        cells[cid] = (top, bottom)

    finalCells = {}
    for cid, (top, bottom) in cells.items():
        polygon = top + bottom[::-1]
        finalCells[cid] = polygon

    return finalCells

def adjustCellEdges(cells, binaryMap):
    for cid, polygon in cells.items():
        maxYForX = {}
        for x, y in polygon:
            if x not in maxYForX or y > maxYForX[x]:
                maxYForX[x] = y

        for i, (x, y) in enumerate(polygon):
            if y == maxYForX[x]:
                polygon[i] = (x, y - 1)

        if len(polygon) >= 2:
            firstX, firstY = polygon[0]
            lastX, lastY = polygon[-1]
            preserveEnds = (
                firstX == 0 and lastX == 0 and
                binaryMap[int(round(firstY)), int(round(firstX))] == 0 and
                binaryMap[int(round(lastY)), int(round(lastX))] == 0
            )
            if not preserveEnds:
                if len(polygon) > 4:
                    polygon.pop(0)
                    polygon.pop(-1)
                else:
                    polygon[0] = (firstX + 1, firstY)
                    polygon[-1] = (lastX + 1, lastY)
            else:
                polygon.insert(0, (firstX, firstY))
                polygon.append((lastX, lastY))

    for cid, polygon in cells.items():
        if len(polygon) > 1 and polygon[0] == polygon[1]:
            polygon.pop(0)
        if len(polygon) > 1 and polygon[-1] == polygon[-2]:
            polygon.pop()

    for cellId, polygon in cells.items():
        cleanedVertices = []
        previousVertex = None
        midIndex = len(polygon) // 2

        for idx, vertex in enumerate(polygon):
            if vertex == previousVertex and idx != midIndex:
                continue

            cleanedVertices.append(vertex)
            previousVertex = vertex

        cells[cellId] = cleanedVertices

    return cells

def findAdjacentCells(cells):
    adjacency = defaultdict(set)
    cellAreas = {}

    for cid, polygon in cells.items():
        occupied = set()
        xs = [x for x, _ in polygon]
        minX, maxX = min(xs), max(xs)

        xToYs = defaultdict(list)
        for x, y in polygon:
            xToYs[int(round(x))].append(int(round(y)))

        for x in range(int(round(minX)), int(round(maxX)) + 1):
            ys = xToYs.get(x, [])
            if len(ys) >= 2:
                top = min(ys)
                bottom = max(ys)
                for y in range(top, bottom + 1):
                    occupied.add((x, y))
        cellAreas[cid] = occupied

    for cid1, area1 in cellAreas.items():
        for cid2, area2 in cellAreas.items():
            if cid1 >= cid2:
                continue
            for x, y in area1:
                if ((x + 1, y) in area2 or (x - 1, y) in area2 or
                    (x, y + 1) in area2 or (x, y - 1) in area2):
                    adjacency[cid1].add(cid2)
                    adjacency[cid2].add(cid1)
                    break

    return adjacency

def getCellCentroids(cells):
    centroids = {}
    for cid, vertices in cells.items():
        xs, ys = zip(*vertices)
        centroid = (sum(xs) / len(xs), sum(ys) / len(ys))
        centroids[cid] = centroid
    return centroids

def buildAdjacencyGraph(centroids, adjacency):
    graph = nx.Graph()
    for cid, center in centroids.items():
        graph.add_node(cid, pos=center)
    for cid1, neighbors in adjacency.items():
        for cid2 in neighbors:
            if cid1 < cid2:
                x1, y1 = centroids[cid1]
                x2, y2 = centroids[cid2]
                dist = np.hypot(x2 - x1, y2 - y1)
                graph.add_edge(cid1, cid2, weight=dist)
    return graph

def getCoverageOrder(graph):
    from networkx.algorithms.approximation import traveling_salesman_problem

    if len(graph.nodes) == 1:
        return [list(graph.nodes)[0]]

    if not nx.is_connected(graph):
        raise ValueError("Graph must be connected for TSP")

    return traveling_salesman_problem(graph, cycle=False, weight='weight')

def saveDecomposition(cells, outputCsv):
    with open(outputCsv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['CellID', 'VertexID', 'X', 'Y'])
        for cellId, vertices in cells.items():
            for vertexId, (x, y) in enumerate(vertices):
                writer.writerow([cellId, vertexId, x, y])

def plotDecomposition(binaryMap, cells, outputImagePath):
    plt.figure(figsize=(10, 8))
    plt.imshow(binaryMap, cmap='gray_r', origin='upper')
    for cellId, vertices in cells.items():
        draw_vertices = vertices[:]

        if len(draw_vertices) == 2:
            draw_vertices = [draw_vertices[0], draw_vertices[1], draw_vertices[1], draw_vertices[0]]

        polygon = MplPolygon(draw_vertices, closed=True, edgecolor='blue', facecolor='none', linewidth=1)
        plt.gca().add_patch(polygon)
        xs, ys = zip(*draw_vertices)
        midX = sum(xs) / len(xs)
        midY = sum(ys) / len(ys)
        plt.text(midX, midY, str(cellId), color='blue', fontsize=12, ha='center', va='center')
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(outputImagePath, dpi=300)
    plt.close()

def saveCoverageOrder(order, outputPath):
    with open(outputPath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(order)

def main():
    parser = argparse.ArgumentParser(description="Perform cellular decomposition on a binary elevation map")
    parser.add_argument("input_csv", help="Input binary elevation map (.csv)")
    parser.add_argument("decomposition_csv", type=str, help="Output cellular decomposition (.csv")
    args = parser.parse_args()

    decompositionCsv = args.decomposition_csv
    decompositionImage = args.decomposition_csv.rsplit(".csv", 1)[0] + ".png"
    traversalOrderPath = args.decomposition_csv.rsplit(".csv", 1)[0] + "_order.csv"
    binaryOutputPath = args.decomposition_csv.rsplit(".csv", 1)[0] + "_binary.csv"

    binaryMap = loadBinaryMap(args.input_csv)

    np.savetxt(binaryOutputPath, binaryMap, delimiter=",", fmt='%d')
    print(f"Binary map saved to: {binaryOutputPath}")

    cells = extractCellsUsingConnectivity(binaryMap)
    cells = adjustCellEdges(cells, binaryMap)
    saveDecomposition(cells, decompositionCsv)
    print(f"Decomposition CSV exported to: {decompositionCsv}")
    plotDecomposition(binaryMap, cells, decompositionImage)
    print(f"Decomposition image saved to: {decompositionImage}")

    adjacency = findAdjacentCells(cells)
    centroids = getCellCentroids(cells)
    graph = buildAdjacencyGraph(centroids, adjacency)
    order = getCoverageOrder(graph)

    saveCoverageOrder(order, traversalOrderPath)
    print(f"Traversal order saved to: {traversalOrderPath}")

if __name__ == "__main__":
    main()