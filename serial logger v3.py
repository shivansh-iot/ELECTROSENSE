import serial
import csv
import sys
import os
import time

# ---------------- SETTINGS ----------------
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

SAVE_FOLDER = r"C:\Users\ACER\OneDrive\Documents\ELECTROSENSE_DATA"
os.makedirs(SAVE_FOLDER, exist_ok=True)

if len(sys.argv) > 1:
    filename = sys.argv[1]
    if not filename.endswith(".csv"):
        filename += ".csv"
else:
    filename = "session_" + time.strftime("%Y%m%d_%H%M%S") + ".csv"

OUTPUT_CSV = os.path.join(SAVE_FOLDER, filename)

# ---- THRESHOLDS — MUST match firmware v2.4 exactly ----
DYNAMIC_RISE_DELTA   = 0.02   # V — same as firmware DYNAMIC_RISE_DELTA
DYNAMIC_SETTLE_DELTA = 0.015  # V — same as firmware DYNAMIC_SETTLE_DELTA
SETTLE_COUNT_REQ     = 3      # same as firmware
CURRENT_TRIGGER_MA   = 0.5    # same as firmware (gCurrent > 0.5f also triggers)
MAX_CYCLE_SAMPLES    = 21     # safety cap == EI window (700ms / 33ms)
BASELINE_RESET_V     = 0.05   # same as firmware idle-reset threshold
MAX_TRIGGER_START_V  = 1.0    # NEW: if voltage is already above this when trigger
                               # fires, it's noise at steady-state (cap already
                               # charged), NOT a genuine 0V->rise event. Reject it.
# --------------------------------------------

print("=" * 50)
print("ELECTROSENSE Serial Logger v3 — Dynamic-Trigger Capture")
print("=" * 50)
print(f"Serial Port : {SERIAL_PORT}")
print(f"Save Path   : {OUTPUT_CSV}")
print("Only records ONE clean rise->settle window per capacitor,")
print("matching firmware v2.4's dynamic delta-based trigger logic.")
print("=" * 50)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"\nERROR: Serial port open nahi hua -> {e}")
    print("Check karo: ESP32 connected hai? Cable thik hai? COM port sahi hai?")
    sys.exit(1)

csv_file = open(OUTPUT_CSV, "a", newline="")
writer = csv.writer(csv_file)

if csv_file.tell() == 0:
    writer.writerow(["timestamp", "voltage", "current", "power", "deltaV"])
    print("Naya file bana, header likh diya.")
else:
    print("Existing file mein data add hoga (append mode).")

print("\nWaiting for capacitor... Press Ctrl+C to stop.\n")

sample_count   = 0
cycle_count    = 0
capturing      = False
cycle_buffer   = []
settle_count   = 0

try:
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        if line.startswith("#") or line.startswith("=") or line.startswith("Captured"):
            print(f"[device] {line}" + " " * 20, end="\r")
            continue

        parts = line.split(",")
        if len(parts) != 5:
            continue

        try:
            ts, v, i, p, dv = (float(parts[0]), float(parts[1]),
                                float(parts[2]), float(parts[3]), float(parts[4]))
        except ValueError:
            continue  # header line ya garbage, skip

        # ── Idle/baseline reset (matches firmware: v < 0.05 and not capturing) ──
        if v < BASELINE_RESET_V and not capturing:
            cycle_buffer = []
            settle_count = 0
            continue

        # ── Dynamic trigger detection (matches firmware exactly) ──
        if not capturing:
            if dv >= DYNAMIC_RISE_DELTA or i > CURRENT_TRIGGER_MA:
                if v > MAX_TRIGGER_START_V:
                    # False trigger — noise while cap already sitting at
                    # steady high voltage. NOT a genuine charge event. Skip.
                    print(f"[skip] Noise-trigger ignored (V={v:.3f} already high){' '*5}")
                    continue
                capturing    = True
                cycle_buffer = []
                settle_count = 0
                print(f"[capture] Rise detected, recording cycle...{' '*10}")
            else:
                continue  # not triggered yet, don't record

        # ── Capture phase ──
        cycle_buffer.append(parts)

        if abs(dv) < DYNAMIC_SETTLE_DELTA:
            settle_count += 1
        else:
            settle_count = 0

        settled     = settle_count >= SETTLE_COUNT_REQ
        buffer_full = len(cycle_buffer) >= MAX_CYCLE_SAMPLES

        if settled or buffer_full:
            for row in cycle_buffer:
                writer.writerow(row)
                sample_count += 1
            csv_file.flush()
            cycle_count += 1
            reason = "settled" if settled else "window full"
            print(f"[done] Cycle #{cycle_count} saved ({len(cycle_buffer)} samples, {reason}). "
                  f"Total samples: {sample_count}{' '*10}")

            capturing    = False
            cycle_buffer = []
            settle_count = 0

except KeyboardInterrupt:
    print(f"\n\nStopped by user.")
    print(f"Total cycles logged : {cycle_count}")
    print(f"Total samples logged: {sample_count}")
    print(f"File saved at: {OUTPUT_CSV}")
finally:
    ser.close()
    csv_file.close()