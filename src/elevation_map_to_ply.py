import numpy as np
import trimesh
import argparse
import os

def elevationMapToMesh(grid):
    rows, cols = grid.shape
    baseVerts = []
    faces = []
    vertIndex = {}

    def getVertIndex(pos):
        key = tuple(pos)
        if key not in vertIndex:
            vertIndex[key] = len(baseVerts)
            baseVerts.append(pos)
        return vertIndex[key]

    for r in range(rows - 1):
        for c in range(cols - 1):
            elevs = [grid[r, c], grid[r, c+1], grid[r+1, c], grid[r+1, c+1]]
            if not all(np.isfinite(e) for e in elevs):
                continue

            v0 = np.array([c,     elevs[0], r])
            v1 = np.array([c + 1, elevs[1], r])
            v2 = np.array([c,     elevs[2], r + 1])
            v3 = np.array([c + 1, elevs[3], r + 1])

            centerPos = (v0 + v1 + v2 + v3) / 4.0

            i0 = getVertIndex(v0)
            i1 = getVertIndex(v1)
            i2 = getVertIndex(v2)
            i3 = getVertIndex(v3)
            ic = getVertIndex(centerPos)

            faces.append([i0, i1, ic])
            faces.append([i1, i3, ic])
            faces.append([i3, i2, ic])
            faces.append([i2, i0, ic])

    if not faces:
        raise ValueError("No valid faces generated from elevation map.")

    return np.array(baseVerts), np.array(faces)

def saveMeshAsPly(vertices, faces, outputPath):
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.export(outputPath)
    print(f"Mesh saved to: {outputPath}")

def main():
    parser = argparse.ArgumentParser(description="Convert an elevation map CSV file into a 3D mesh (.ply).")
    parser.add_argument("input_csv", help="Input .csv elevation map file")
    parser.add_argument("output_ply", help="Output .ply mesh file")
    args = parser.parse_args()

    if not args.input_csv.lower().endswith('.csv'):
        raise ValueError("Input file must have a .csv extension")
    if not args.output_ply.lower().endswith('.ply'):
        raise ValueError("Output file must have a .ply extension")
    if not os.path.exists(args.input_csv):
        raise FileNotFoundError(f"CSV file not found: {args.input_csv}")

    grid = np.genfromtxt(args.input_csv, delimiter=',')

    vertices, faces = elevationMapToMesh(grid)
    saveMeshAsPly(vertices, faces, args.output_ply)

if __name__ == "__main__":
    main()