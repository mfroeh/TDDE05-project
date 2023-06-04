import csv
import sys

def divide_first_column(filename):
    # Open the CSV file for reading and writing
    with open(filename, 'r') as file:
        # Create a CSV reader object
        reader = csv.reader(file)

        # Create a list to store the modified rows
        modified_rows = []

        start = 0
        # Iterate over each row in the CSV file
        for row in reader:
            # Check if the row has at least one value
            if row and not row[0] == "T":
                if not start:
                    start = row[0]
                
                # Divide the first value by 10^9 and update the row
                first_value = float(int(row[0]) - int(start)) / 10**9
                row[0] = str(first_value)

            # Add the modified row to the list
            modified_rows.append(row)

    # Write the modified rows back to the CSV file
    with open("clean" + filename, 'w', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)

        # Write the modified rows to the file
        writer.writerows(modified_rows)

# Check if a filename is provided as a command-line argument
if len(sys.argv) < 2:
    print("Please provide a filename as a command-line argument.")
    sys.exit(1)

# Get the filename from the command-line argument
filename = sys.argv[1]

# Call the function to divide the first column by 10^9
divide_first_column(filename)
