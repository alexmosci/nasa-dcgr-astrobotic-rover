import argparse
import csv

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_csv", type=str, help="Path to output binary map CSV file")
    parser.add_argument('--dimensions', type=str, default="1,1", help="Map dimensions as x,y (default: 1,1)")
    args = parser.parse_args()

    try:
        xStr, yStr = args.dimensions.split(",")
        x, y = int(xStr), int(yStr)
    except:
        raise argparse.ArgumentTypeError("Invalid format for --dimensions. Expected x,y with integers")

    with open(args.output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        for _ in range(y):
            writer.writerow([0] * x)

if __name__ == "__main__":
    main()