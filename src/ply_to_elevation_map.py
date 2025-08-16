import numpy as np
import trimesh
import argparse
import os
import matplotlib.pyplot as plt

def loadPly(plyPath):
    mesh = trimesh.load(plyPath, process=False)

    if hasattr(mesh, 'vertices') and mesh.vertices.shape[1] == 3:
        return mesh.vertices

    if hasattr(mesh, 'geometry') and len(mesh.geometry) > 0:
        for geo in mesh.geometry.values():
            if hasattr(geo, 'vertices') and geo.vertices.shape[1] == 3:
                return geo.vertices

    raise ValueError("Could not find valid 3D vertices in the .ply file.")

def cleanExtremeValues(points, axisIndex, minVal=-1000, maxVal=1000):
    cleaned = points.copy()
    values = cleaned[:, axisIndex]
    mask = (values < minVal) | (values > maxVal) | ~np.isfinite(values)
    cleaned[mask, axisIndex] = 0.0
    return cleaned

def createElevationMap(points, resolution=100, elevationAxis='y'):
    axisMap = {'x': 0, 'y': 1, 'z': 2}
    elevIndex = axisMap[elevationAxis]

    horizAxes = [i for i in range(3) if i != elevIndex]
    axis1Index, axis2Index = horizAxes

    axis1Vals = points[:, axis1Index]
    axis2Vals = points[:, axis2Index]
    elevationVals = points[:, elevIndex]

    min1, max1 = axis1Vals.min(), axis1Vals.max()
    min2, max2 = axis2Vals.min(), axis2Vals.max()
    
    range1 = max1 - min1
    range2 = max2 - min2

    if range1 >= range2:
        gridWidth = resolution
        gridHeight = int(resolution * (range2 / range1))
    else:
        gridHeight = resolution
        gridWidth = int(resolution * (range1 / range2))

    gridWidth = max(1, gridWidth)
    gridHeight = max(1, gridHeight)

    binSize1 = range1 / gridWidth
    binSize2 = range2 / gridHeight

    binScale = (binSize1 + binSize2) / 2
    elevationVals = elevationVals / binScale

    grid = np.full((gridHeight, gridWidth), np.nan)
    count = np.zeros((gridHeight, gridWidth), dtype=int)

    for a1, elev, a2 in zip(axis1Vals, elevationVals, axis2Vals):
        col = int((a1 - min1) / binSize1)
        row = int((a2 - min2) / binSize2)

        col = min(col, gridWidth - 1)
        row = min(row, gridHeight - 1)

        if np.isnan(grid[row, col]):
            grid[row, col] = elev
        else:
            grid[row, col] += elev

        count[row, col] += 1

    with np.errstate(invalid='ignore'):
        grid = np.divide(grid, count, where=count > 0)

    return grid

def smoothGrid(grid, passes=0):
    filled = grid.copy()
    rows, cols = filled.shape

    for _ in range(passes):
        newGrid = filled.copy()
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                val = filled[r, c]
                if val == 0 or np.isnan(val):
                    neighbors = []

                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            if dr == 0 and dc == 0:
                                continue
                            nr, nc = r + dr, c + dc
                            neighborVal = filled[nr, nc]
                            if neighborVal != 0 and not np.isnan(neighborVal):
                                neighbors.append(neighborVal)

                    if len(neighbors) >= 3:
                        newGrid[r, c] = np.mean(neighbors)
        filled = newGrid

    return filled

def maskEdgeConnectedZeros(grid):
    grid = grid.copy()
    visited = np.zeros_like(grid, dtype=bool)
    rows, cols = grid.shape
    stack = []

    for c in range(cols):
        if grid[0, c] == 0:
            stack.append((0, c))
        if grid[rows - 1, c] == 0:
            stack.append((rows - 1, c))
    for r in range(rows):
        if grid[r, 0] == 0:
            stack.append((r, 0))
        if grid[r, cols - 1] == 0:
            stack.append((r, cols - 1))

    while stack:
        r, c = stack.pop()
        if visited[r, c] or grid[r, c] != 0:
            continue
        visited[r, c] = True
        grid[r, c] = np.nan
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if not visited[nr, nc] and grid[nr, nc] == 0:
                    stack.append((nr, nc))

    return grid

def patchCorners(grid):
    rows, cols = grid.shape

    def is_nan(r, c):
        return 0 <= r < rows and 0 <= c < cols and np.isnan(grid[r, c])

    def is_zero(r, c):
        return 0 <= r < rows and 0 <= c < cols and np.isclose(grid[r, c], 0.0, atol=1e-8)

    trigger = is_nan(1, 1) or is_nan(1, 0) or is_nan(0, 2)

    if trigger:
        if is_zero(0, 0):
            grid[0, 0] = np.nan
        if is_zero(0, 1):
            grid[0, 1] = np.nan

    return grid

def saveToCsv(grid, outputPath):
    np.savetxt(outputPath, grid, delimiter=',', fmt='%.3f')

def savePng(grid, outputCsvPath):
    pngPath = outputCsvPath.replace('.csv', '.png')
    plt.figure(figsize=(10, 8))
    cmap = plt.get_cmap('terrain')
    cmap.set_bad(color='white')

    maskedGrid = np.ma.masked_invalid(grid)
    plt.imshow(maskedGrid, cmap=cmap, origin='lower')
    plt.colorbar(label='Elevation')
    plt.title("Elevation Map")
    plt.axis('off')

    plt.savefig(pngPath, bbox_inches='tight', dpi=300)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Convert a .ply mesh file into a 2D elevation map (.csv + .png).")
    parser.add_argument("input_ply", help="Input .ply mesh file")
    parser.add_argument("output_csv", help="Output .csv elevation map file")
    parser.add_argument("--resolution", type=int, default=100,
                        help="Number of pixels along the longer axis (default: 100)")
    parser.add_argument("--elevation_axis", choices=['x', 'y', 'z'], default='y',
                        help="Axis to treat as elevation: 'x', 'y', or 'z' (default: 'y')")
    parser.add_argument("--smooth_passes", type=int, default=0,
                        help="Number of smoothing passes to fill interior gaps (default: 0)")
    args = parser.parse_args()

    if not args.input_ply.lower().endswith('.ply'):
        raise ValueError("Input file must have a .ply extension")
    if not args.output_csv.lower().endswith('.csv'):
        raise ValueError("Output file must have a .csv extension")
    if not os.path.exists(args.input_ply):
        raise FileNotFoundError(f"PLY file not found: {args.input_ply}")

    axisIndex = {'x': 0, 'y': 1, 'z': 2}[args.elevation_axis]

    points = loadPly(args.input_ply)
    points = cleanExtremeValues(points, axisIndex)

    print("Axis ranges:")
    print(f"X: {points[:, 0].min():.2f} to {points[:, 0].max():.2f}")
    print(f"Y: {points[:, 1].min():.2f} to {points[:, 1].max():.2f}")
    print(f"Z: {points[:, 2].min():.2f} to {points[:, 2].max():.2f}")

    grid = createElevationMap(points, args.resolution, args.elevation_axis)
    grid = smoothGrid(grid, passes=args.smooth_passes)
    grid = maskEdgeConnectedZeros(grid)
    grid = patchCorners(grid)

    saveToCsv(grid, args.output_csv)
    savePng(grid, args.output_csv)

    print(f"Elevation map exported: {args.output_csv} and {args.output_csv.replace('.csv', '.png')}")
    print(f"Grid shape: {grid.shape} (rows x cols)")

if __name__ == "__main__":
    main()