import serial
import time
import random

# Configure the serial port (replace with your port, e.g., /dev/pts/4)
ser = serial.Serial('/dev/pts/4', 115200, timeout=1)

# Define a function to simulate responses based on the command received
def handle_command(command):
    if command.startswith("G28"):
        return "ok\n"  # Homing command
    elif command.startswith("M1000"):
        return "SYNC\n"  # Sync command
    elif command.startswith("G0"):
        return "ok\n"  # Move command
    elif command.startswith("GM201"):
        return "ok\n"  # Set acceleration
    elif command.startswith("GM203"):
        return "ok\n"  # Set max feedrate
    elif command.startswith("M204"):
        return "ok\n"  # Set feedrate multiplier
    elif command.startswith("G4T"):
        return "ok\n"  # Dwell command
    elif command.startswith("M10"):
        return "ok\n"  # Turn vacuum pump on
    elif command.startswith("M11"):
        return "ok\n"  # Turn vacuum pump off
    elif command.startswith("M126"):
        return "ok\n"  # Turn vacuum valve on
    elif command.startswith("M127"):
        return "ok\n"  # Turn vacuum valve off
    elif command.startswith("M17"):
        return "ok\n"  # Turn steppers on
    elif command.startswith("M18"):
        return "ok\n"  # Turn steppers off
    elif command.startswith("M42"):
        return "ok\n"  # IO control
    elif command.startswith("M205"):
        return "ok\n"  # Feeder advance
    elif command.startswith("M512"):
        return "ok\n"  # Default settings
    else:
        return "ERR_COMMAND_NOT_FOUND\n"

def simulate_pnp():
    print("Starting PnP simulation...")
    while True:
        if ser.in_waiting > 0:
            command = ser.readline().decode('utf-8').strip()
            print(f"Received command: {command}")

            # Simulate response
            response = handle_command(command)

            # Simulate potential delays or processing time
            time.sleep(random.uniform(0.05, 0.2))  # Add some randomness to response time

            ser.write(response.encode('utf-8'))
            print(f"Sent response: {response}")

        time.sleep(0.1)

if __name__ == "__main__":
    simulate_pnp()
