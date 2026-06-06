#PLOTTER PYTHON

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

    chosen = ports[0].device
    print(f"\n Using {chosen}\n")
    return chosen


SERIAL_PORT = find_serial_port()
BAUD_RATE = 500000
filename = 'data_log.txt'

if SERIAL_PORT is None:
    exit(1)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

print(
    f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...\n"
    f"Commands: <REB>, <ARM>, MAN/AUTO, AZxxx, ELxx, Dxxx\n"
)

# -----------------------------
# Function: Listen & Log
# -----------------------------
# -----------------------------
# SERIAL LOGGING LOOP (SINGLE THREAD)
# -----------------------------
def read_serial():
    with open('data_log.txt', "a") as f:
        print(f"[LOGGING] Writing to {'data_log.txt'}")

        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                log_entry = f"[{timestamp}] {line}"

                print(log_entry)
                f.write(log_entry + "\n")
                f.flush()

        except KeyboardInterrupt:
            print("\n[STOPPED] Logging interrupted by user.")

        except serial.SerialException:
            print("[ERROR] Serial connection lost.")

        finally:
            ser.close()


# -----------------------------
# RUN (NO THREADS)
# -----------------------------
try:
    read_serial()

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    try:
        ser.close()
    except:
        pass
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