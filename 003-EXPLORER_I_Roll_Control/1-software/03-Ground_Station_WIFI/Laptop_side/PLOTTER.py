# PLOTTER PYTHON

import serial
import serial.tools.list_ports
import time
import threading

# -----------------------------
# Find available serial port
# -----------------------------
def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found...")
        return None
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} ({p.description})")
    # Pick first port automatically
    chosen = ports[0].device
    print(f"\n Using {chosen}\n")
    return chosen

SERIAL_PORT = find_serial_port()
BAUD_RATE = 500000
LOG_FILE = "serial_log.txt"

if SERIAL_PORT is None:
    exit(1)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...\n"
      f"Type commands (<REB> to reboot ; <ARM> to arm the rocket) and press Enter.\n"
      f"Press Ctrl+C to exit.\n")

# -----------------------------
# Function: Listen & Log
# -----------------------------
def read_serial():
    with open(LOG_FILE, "a") as f:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                    log_entry = f"[{timestamp}] {line}"
                    print(log_entry)
                    f.write(log_entry + "\n")
                    f.flush()

# -----------------------------
# Function: Send Commands
# -----------------------------
def send_commands():
    while True:
        try:
            cmd = input().strip()
            if cmd:
                ser.write((cmd + "\n").encode('utf-8'))
                print(f">>> Sent: {cmd}")
        except EOFError:
            break

# -----------------------------
# Run threads
# -----------------------------
try:
    t1 = threading.Thread(target=read_serial, daemon=True)
    t1.start()
    send_commands()
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
