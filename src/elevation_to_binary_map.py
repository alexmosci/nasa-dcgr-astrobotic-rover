import numpy as np
import argparse
import os
import matplotlib.pyplot as plt
from collections import deque

def smoothGrid(binary, passes=0):
    filled = binary.copy()
    rows, cols = filled.shape

    for _ in range(passes):
        newBinary = filled.copy()
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if filled[r, c] != 1:
                    continue
                if np.isnan(globalElevation[r, c]):
                    continue
                wall_neighbors = 0
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if filled[r + dr, c + dc] == 1:
                        wall_neighbors += 1
                if wall_neighbors <= 2:
                    newBinary[r, c] = 0
        filled = newBinary
    return filled

def generateBinaryMap(grid, threshold=0.5):
    rows, cols = grid.shape
    binary = np.zeros_like(grid, dtype=int)

    for r in range(rows):
        for c in range(cols):
            val = grid[r, c]

            if np.isnan(val):
                binary[r, c] = 1
                continue

            max_diff = 0.0

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    neighbor = grid[nr, nc]
                    if not np.isnan(neighbor):
                        diff = abs(val - neighbor)
                        max_diff = max(max_diff, diff)

            if max_diff > threshold:
                binary[r, c] = 1

    fillInteriorIslands(binary)
    return binary

def fillInteriorIslands(binary):
    rows, cols = binary.shape
    visited = np.zeros_like(binary, dtype=bool)
    queue = deque()

    for r in range(rows):
        for c in [0, cols - 1]:
            if binary[r, c] == 0 and not visited[r, c]:
                queue.append((r, c))
                visited[r, c] = True
    for c in range(cols):
        for r in [0, rows - 1]:
            if binary[r, c] == 0 and not visited[r, c]:
                queue.append((r, c))
                visited[r, c] = True

    while queue:
        r, c = queue.popleft()
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if binary[nr, nc] == 0 and not visited[nr, nc]:
                    visited[nr, nc] = True
                    queue.append((nr, nc))

    for r in range(rows):
        for c in range(cols):
            if binary[r, c] == 0 and not visited[r, c]:
                binary[r, c] = 1

def visualizeBinaryMap(binaryMap, outputPath):
    plt.figure(figsize=(10, 8))
    cmap = plt.get_cmap('gray_r')
    plt.imshow(binaryMap, cmap=cmap, origin='lower')
    plt.title("Binary Traversability Map (0 = go, 1 = wall)")
    plt.axis('off')
    pngPath = outputPath.replace('.csv', '.png')
    plt.savefig(pngPath, bbox_inches='tight', dpi=300)
    plt.close()
    print(f"Binary PNG saved to: {pngPath}")

def main():
    parser = argparse.ArgumentParser(description="Convert an elevation grid to binary map based on threshold.")
    parser.add_argument("input_csv", help="Input .csv elevation map file")
    parser.add_argument("output_csv", help="Output .csv binary map file")
    parser.add_argument("--threshold", type=float, default=0.5,
                        help="Height difference threshold to mark areas as walls (default: 0.5)")
    parser.add_argument("--smooth_passes", type=int, default=0,
                        help="Number of smoothing passes after binary conversion (default: 0)")
    args = parser.parse_args()

    if not args.input_csv.lower().endswith('.csv'):
        raise ValueError("Input file must be a .csv file")
    if not args.output_csv.lower().endswith('.csv'):
        raise ValueError("Output file must be a .csv file")
    if not os.path.exists(args.input_csv):
        raise FileNotFoundError(f"Input CSV not found: {args.input_csv}")

    elevationGrid = np.genfromtxt(args.input_csv, delimiter=',')
    global globalElevation
    globalElevation = elevationGrid
    binaryMap = generateBinaryMap(elevationGrid, args.threshold)
    binaryMap = smoothGrid(binaryMap, passes=args.smooth_passes)

    np.savetxt(args.output_csv, binaryMap, delimiter=',', fmt='%d')
    print(f"Binary map saved to: {args.output_csv}")

    visualizeBinaryMap(binaryMap, args.output_csv)

if __name__ == "__main__":
    main()