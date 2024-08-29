import serial
import csv
import time

# Configure serial port
serial_port = 'COM5'  # Replace with your serial port
baud_rate = 57600  # Replace with your baud rate
timeout = 1  # Timeout for serial read

# Open serial port
ser = serial.Serial(serial_port, baud_rate, timeout=timeout)

# CSV file name
csv_file = 'Delay 100 Avg 100 Prop.csv'

# Open CSV file in append mode
with open(csv_file, mode='a', newline='') as file:
    writer = csv.writer(file)

    # Infinite loop to read from serial port
    while True:
        try:
            # Read data from serial port
            line = ser.readline().decode('utf-8', errors='replace').strip()

            if line:
                # Split the line into data elements if necessary
                data = line.split(',')

                # Write data to CSV file
                writer.writerow(data)
                print(f"Data written to CSV: {data}")

        except KeyboardInterrupt:
            # Exit the loop if the user presses Ctrl+C
            print("Program terminated by user.")
            break

# Close serial port
ser.close()
