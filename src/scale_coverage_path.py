import argparse
import csv

def main():
    parser = argparse.ArgumentParser(description="Scale X and Y coordinates in a coverage path CSV.")
    parser.add_argument("input_path", help="Input coverage path CSV file")
    parser.add_argument("output_path", help="Output scaled coverage path CSV file")
    parser.add_argument("--scale", type=str, default="1", help="Scale factor for real-world units. Can be a fraction (default: 1)")
    args = parser.parse_args()

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

    scaledPath = []

    with open(args.input_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader, None)
        for row in reader:
            if not row:
                continue
            try:
                order = int(row[0])
                x = float(row[1]) * scaleFactor
                y = float(row[2]) * scaleFactor
                heading = float(row[3])
                scaledPath.append((order, x, y, heading))
            except (ValueError, IndexError):
                continue

    with open(args.output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Order', 'X', 'Y', 'Heading'])
        writer.writerows(scaledPath)

    print(f"Scaled path saved to: {args.output_path}")

if __name__ == "__main__":
    main()