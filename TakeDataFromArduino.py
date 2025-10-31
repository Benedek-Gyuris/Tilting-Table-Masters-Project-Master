import serial
import time
import pandas as pd

# Configure serial port
port = 'COM7'  # Change this to your port
baud_rate = 115200  # Ensure this matches the Arduino sketch
timeout = 1  # Timeout in seconds

# Open serial port
ser = serial.Serial(port, baud_rate, timeout=timeout)
time.sleep(2)  # Wait for the serial connection to initialize

# Create an empty list to store the data
data = []

try:
    print("Collecting data...")
    while True:
        if ser.in_waiting > 0:  # Check if data is available
            line = ser.readline().decode('utf-8').strip()  # Read and decode the line
            values = line.split(',')
            if len(values) == 5:  # Ensure we have exactly 5 values
                try:
                    # Parse and convert the values to floats
                    parsed_values = [float(val) for val in values]
                    data.append(parsed_values)  # Append to the data list
                except ValueError:
                    print("Error parsing values:", values)
except KeyboardInterrupt:
    print("Terminating data collection.")
finally:
    ser.close()

# Convert the collected data to a DataFrame
columns = ['Value1', 'Value2', 'Value3', 'Value4', 'Value5']  # Rename as needed
df = pd.DataFrame(data, columns=columns)

# Save the data to a CSV file (optional)
df.to_csv('arduino_data.csv', index=False)
print("Data saved to arduino_data.csv")

# Display the first few rows of the DataFrame
print(df.head())
