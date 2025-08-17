import argparse
import csv
import sys

def flipHorizontal(x, y, heading):
    return -x, y, (360.0 - heading) % 360.0

def flipVertical(x, y, heading):
    return x, -y, (180.0 - heading) % 360.0

def main():
    parser = argparse.ArgumentParser(description="Flip a coverage path based on map dimensions")
    parser.add_argument("input_path", help="Input coverage path CSV")
    parser.add_argument("output_path", help="Output coverage path CSV")
    parser.add_argument("--dimensions", default=None, help="Map dimensions as x,y. Defaults to inferring from coverage path")
    args = parser.parse_args()

    try:
        with open(args.input_path, newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            rows = [r for r in reader if r]
    except FileNotFoundError:
        print(f"Input file not found: {args.input_path}", file=sys.stderr)
        sys.exit(1)

    parsed = []
    for r in rows:
        if len(r) < 4:
            continue
        try:
            order = int(r[0])
            x = float(r[1])
            y = float(r[2])
            heading = float(r[3])
            parsed.append((order, x, y, heading))
        except Exception:
            continue

    if not parsed:
        print("No valid rows found", file=sys.stderr)
        sys.exit(1)

    if args.dimensions is None:
        xs = [x for _, x, _, _ in parsed]
        ys = [y for _, _, y, _ in parsed]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
    else:
        try:
            wStr, hStr = args.dimensions.split(",")
            width = float(wStr.strip())
            height = float(hStr.strip())
        except Exception:
            print("Invalid --dimensions. Expected x,y", file=sys.stderr)
            sys.exit(2)

    flipMode = "horizontal" if width > height else "vertical"

    outRows = []
    for order, x, y, heading in parsed:
        if flipMode == "horizontal":
            x, y, heading = flipHorizontal(x, y, heading)
        else:
            x, y, heading = flipVertical(x, y, heading)
        outRows.append((order, x, y, heading))

    with open(args.output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(['Order', 'X', 'Y', 'Heading'])
        writer.writerows(outRows)

    print(f"{flipMode.capitalize()} flip applied using dimensions {width},{height}.")
    print(f"Saved: {args.output_path}")

if __name__ == "__main__":
    main()