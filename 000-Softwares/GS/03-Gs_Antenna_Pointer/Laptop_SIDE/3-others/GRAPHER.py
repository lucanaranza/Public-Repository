import serial
import serial.tools.list_ports
import time
import threading
import matplotlib.pyplot as plt
from collections import deque

# -----------------------------
# DATA
# -----------------------------
MAX_POINTS = 200

alt = deque(maxlen=MAX_POINTS)
vz = deque(maxlen=MAX_POINTS)
az = deque(maxlen=MAX_POINTS)
el = deque(maxlen=MAX_POINTS)

lock = threading.Lock()

# -----------------------------
# PARSER
# -----------------------------
def parse_line(line):
    try:
        parts = line.split("|")
        if len(parts) < 2:
            return None

        data = parts[1].split(",")

        return {
            "az": float(data[0]),
            "el": float(data[1]),
            "alt": float(data[16]),
            "vz": float(data[17]),
        }
    except:
        return None


# -----------------------------
# SERIAL PORT
# -----------------------------
def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found")
        exit(1)

    for i, p in enumerate(ports):
        print(f"{i}: {p.device}")

    return ports[int(input("Select port: "))].device


SERIAL_PORT = find_serial_port()
ser = serial.Serial(SERIAL_PORT, 500000, timeout=1)

# -----------------------------
# SERIAL THREAD (ONLY DATA)
# -----------------------------
def serial_loop():
    with open("data_log.txt", "a") as f:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            print(line)
            if not line:
                continue

            f.write(line + "\n")
            f.flush()

            parsed = parse_line(line)
            if parsed:
                with lock:
                    az.append(parsed["az"])
                    el.append(parsed["el"])
                    alt.append(parsed["alt"])
                    vz.append(parsed["vz"])


# -----------------------------
# START SERIAL THREAD
# -----------------------------
threading.Thread(target=serial_loop, daemon=True).start()

# -----------------------------
# GUI (MAIN THREAD - IMPORTANT)
# -----------------------------
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1)

line_alt, = ax1.plot([], [], label="Altitude")
line_vz, = ax1.plot([], [], label="Vz")

line_az, = ax2.plot([], [], label="Az")
line_el, = ax2.plot([], [], label="El")

ax1.legend()
ax2.legend()

ax1.set_title("Flight Data")
ax2.set_title("Ground Station")

print("Running GUI...")

# -----------------------------
# MAIN LOOP (GUI OWNER)
# -----------------------------
while True:
    with lock:
        if len(alt) > 5:
            x = range(len(alt))

            line_alt.set_data(x, list(alt))
            line_vz.set_data(x, list(vz))

            line_az.set_data(range(len(az)), list(az))
            line_el.set_data(range(len(el)), list(el))

            ax1.relim()
            ax1.autoscale_view()

            ax2.relim()
            ax2.autoscale_view()

    plt.pause(0.05)