"""
Real-time Serial Plotter, CSV Logger, and Clipboard Exporter for Arduino/PlatformIO.

Features:
- Connects to Arduino serial port (auto-detects or accepts --port).
- Expects single or comma-separated numerical readings on each line (e.g., angle, or target,current).
- Configurable Signal Labels: Easily set column/plot names in the SIGNAL_LABELS list.
  If a column is not labeled, it automatically falls back to "Signal N".
- Robust to Uploads: Automatically detects disconnects during firmware flashing,
  releases the COM port so the uploader can access it, and automatically
  re-attempts connection and resumes streaming when the board reboots.
- SEPARATE CSV PER RUN: Each upload/reconnection session is automatically logged into
  its own distinct CSV file in logs/ (e.g., session_run01_..., session_run02_...).
- Auto Clipboard: When a run completes (due to upload or exit), that run's entire CSV
  is automatically copied to your clipboard ready to paste into Excel/MATLAB.
- Press [Space] or [P] in the plot window to manually Pause / Release the COM port for upload.
"""

import sys
import os
import time
import argparse
import threading
import queue
import subprocess
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ================= Configuration =================
DEFAULT_BAUD = 115200
DEFAULT_PORT = "COM5"
MAX_PLOT_POINTS = 300  # Number of samples visible in rolling plot
UPDATE_INTERVAL_MS = 30  # Plot refresh rate (~33 FPS)
RECONNECT_DELAY_S = 0.5  # Polling interval during upload/reconnect
MAX_LOG_ROWS = 5000  # Ceiling to prevent memory / clipboard crash (5k rows max per run)

# ================= Signal Labels =================
# Input strings in order for each incoming comma-separated column.
# If fewer labels are provided than incoming data columns, remaining columns
# will automatically fall back to "Signal_N" (or "Signal N" in the plot).
SIGNAL_LABELS = [
    "Current_Angle",
    "Target_Angle",
    "Voltage_Command",
]


def resolve_label(index: int, custom_labels: list = None) -> str:
    """Return the label for a channel index, falling back to 'Signal_<N>' if unlabelled."""
    labels = custom_labels if (custom_labels is not None) else SIGNAL_LABELS
    if labels and index < len(labels) and str(labels[index]).strip():
        return str(labels[index]).strip()
    return f"Signal_{index + 1}"


def find_default_port():
    """Auto-detect connected Arduino / USB serial port."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "uno" in desc or "usb serial" in desc or "vid:pid=2341" in hwid:
            return p.device
    if ports:
        return ports[0].device
    return DEFAULT_PORT


def copy_to_clipboard(text: str):
    """Copy text to clipboard natively on Windows, with cross-platform fallback."""
    # Method 1: Windows native clip.exe
    if sys.platform == "win32":
        try:
            proc = subprocess.Popen(["clip"], stdin=subprocess.PIPE, text=True, close_fds=True)
            proc.communicate(input=text)
            return True
        except Exception:
            pass

    # Method 2: tkinter fallback
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        root.clipboard_clear()
        root.clipboard_append(text)
        root.update()
        root.destroy()
        return True
    except Exception:
        pass

    # Method 3: pyperclip fallback if installed
    try:
        import pyperclip
        pyperclip.copy(text)
        return True
    except Exception:
        pass

    return False


class SerialDataLogger:
    def __init__(self, port, baud, labels=None, max_rows=MAX_LOG_ROWS):
        self.target_port = port
        self.baud = baud
        self.labels = labels if labels is not None else SIGNAL_LABELS
        self.max_rows = max_rows
        self.max_rows_reached = False
        self.serial_conn = None
        self.running = False
        self.paused = False
        self.status_message = "Initializing..."
        self.thread = None
        self.data_queue = queue.Queue()

        # Session tracking
        os.makedirs("logs", exist_ok=True)
        self.run_number = 0
        self.csv_file = None
        self.csv_filename = None
        self.headers_written = False
        self.logged_rows = []
        self.run_start_time = None

    def start(self):
        self.running = True
        print(f"[+] Starting Serial Logger for target port '{self.target_port}'...")
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def toggle_pause(self):
        """Toggle manual pause to explicitly release the COM port for an upload."""
        self.paused = not self.paused
        if self.paused:
            self._close_serial()
            self._finalize_current_run()
            self.status_message = "PAUSED (Port Released for Upload - Press Space to Resume)"
            print("\n[!] Manual Pause: COM port closed and released. Ready for upload.")
            print("[!] Press [Space] again in the plot window to reconnect and resume.")
        else:
            self.status_message = "Resuming connection..."
            print("\n[+] Resuming: Reconnecting to serial port...")

    def _start_new_run(self):
        """Start a brand new logging run and CSV file."""
        self._finalize_current_run()

        self.run_number += 1
        self.max_rows_reached = False
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join("logs", f"session_run{self.run_number:02d}_{timestamp_str}.csv")
        self.csv_file = open(self.csv_filename, "w", encoding="utf-8")
        self.headers_written = False
        self.logged_rows = []
        self.run_start_time = time.time()

        # Signal the plot to clear buffers for this fresh run
        self.data_queue.put({"event": "NEW_RUN", "run_number": self.run_number})
        print(f"\n[+] Started Run #{self.run_number}: Logging to {self.csv_filename}")

    def _finalize_current_run(self):
        """Finalize the active CSV file, copy its content to clipboard, and print summary."""
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

        if self.csv_filename and os.path.exists(self.csv_filename):
            if len(self.logged_rows) == 0:
                # Remove empty file if no data was collected
                try:
                    os.remove(self.csv_filename)
                except Exception:
                    pass
                self.csv_filename = None
                return

            with open(self.csv_filename, "r", encoding="utf-8") as f:
                content = f.read()

            # Ensure clipboard payload never exceeds max ceiling
            lines = content.splitlines()
            if len(lines) > self.max_rows + 1:
                content = "\n".join(lines[: self.max_rows + 1]) + "\n"

            copied = copy_to_clipboard(content)
            print("\n" + "=" * 65)
            print(f"[✓] Run #{self.run_number} finished. Total data rows saved: {len(self.logged_rows)}")
            print(f"[✓] Saved to: {os.path.abspath(self.csv_filename)}")
            if copied:
                print(f"[✓] Run #{self.run_number} CSV data copied to your CLIPBOARD!")
            else:
                print(f"[!] Could not copy to clipboard automatically.")
            print("=" * 65)

            self.csv_filename = None

    def _close_serial(self):
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    def _worker_loop(self):
        """Background thread handling continuous connection, reading, and auto-reconnect."""
        reconnect_attempts = 0

        while self.running:
            # 1. Check if user paused manually
            if self.paused:
                self._close_serial()
                time.sleep(0.1)
                continue

            # 2. If not connected, attempt connection / reconnection
            if self.serial_conn is None or not self.serial_conn.is_open:
                active_ports = [p.device for p in serial.tools.list_ports.comports()]
                target = self.target_port if self.target_port in active_ports else find_default_port()

                try:
                    reconnect_attempts += 1
                    self.status_message = f"Connecting to {target} (attempt #{reconnect_attempts})..."
                    conn = serial.Serial(target, self.baud, timeout=0.5)
                    try:
                        conn.dtr = False
                        time.sleep(0.05)
                        conn.dtr = True
                        conn.reset_input_buffer()
                    except Exception:
                        pass

                    self.serial_conn = conn
                    self.target_port = target
                    reconnect_attempts = 0
                    self.status_message = f"CONNECTED: {target} @ {self.baud} (Run #{self.run_number + 1})"
                    print(f"\n[+] Connected to {target} @ {self.baud} baud!")

                    # Start fresh CSV run for this new connection
                    self._start_new_run()

                except (serial.SerialException, OSError, PermissionError):
                    # Port is in use by uploader, or board is still resetting
                    self.status_message = f"WAITING FOR BOARD / UPLOAD... ({target})"
                    self._close_serial()
                    time.sleep(RECONNECT_DELAY_S)
                    continue

            # 3. Read data from active serial port
            try:
                raw_bytes = self.serial_conn.readline()
                if not raw_bytes:
                    continue

                line = raw_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Parse comma-separated numeric values
                parts = [p.strip() for p in line.split(",") if p.strip()]
                vals = []
                for p in parts:
                    try:
                        vals.append(float(p))
                    except ValueError:
                        break

                if not vals or len(vals) != len(parts):
                    # Line was non-numeric text (e.g. boot message "geeWhiz Started")
                    print(f"[Serial Info] {line}")
                    continue

                now = time.time() - (self.run_start_time if self.run_start_time else time.time())
                row = [now] + vals

                # Log to CSV up to max ceiling
                if len(self.logged_rows) < self.max_rows:
                    if not self.headers_written and self.csv_file:
                        col_headers = [resolve_label(i, self.labels) for i in range(len(vals))]
                        header_line = "Time_s," + ",".join(col_headers) + "\n"
                        self.csv_file.write(header_line)
                        self.headers_written = True

                    csv_line = f"{now:.4f}," + ",".join(f"{v:.4f}" for v in vals) + "\n"
                    if self.csv_file:
                        self.csv_file.write(csv_line)
                        self.csv_file.flush()

                    self.logged_rows.append(csv_line)
                elif not self.max_rows_reached:
                    self.max_rows_reached = True
                    print(f"\n[!] Run #{self.run_number}: Reached max ceiling of {self.max_rows} rows. CSV logging capped to protect clipboard/system.")
                    if self.csv_file and not self.csv_file.closed:
                        self.csv_file.write(f"# --- Max ceiling of {self.max_rows} rows reached ---\n")
                        self.csv_file.flush()

                # Live plot queue always receives data so visualization stays alive
                self.data_queue.put(row)

            except (serial.SerialException, OSError, PermissionError) as e:
                # Board disconnected or reset due to upload!
                print(f"\n[!] Board disconnected / upload detected ({e}).")
                print("[!] Finalizing previous run and waiting for board reboot...")
                self._close_serial()
                self._finalize_current_run()
                self.status_message = "DISCONNECTED (Upload in progress / Waiting for board...)"
                time.sleep(1.0)

    def stop(self):
        if not self.running:
            return
        self.running = False
        self._close_serial()
        self._finalize_current_run()


def run_plotter(port, baud, labels=None, max_rows=MAX_LOG_ROWS):
    logger = SerialDataLogger(port, baud, labels=labels, max_rows=max_rows)
    logger.start()

    # Matplotlib Setup
    plt.style.use("fast")
    fig, ax = plt.subplots(figsize=(10, 5.5))
    fig.canvas.manager.set_window_title(f"Live Serial Plotter - {port}")

    time_buf = deque(maxlen=MAX_PLOT_POINTS)
    lines = []
    data_bufs = []

    def reset_plot_axes():
        nonlocal lines, data_bufs
        lines.clear()
        data_bufs.clear()
        time_buf.clear()
        ax.cla()
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Reading")

    reset_plot_axes()

    def init_plot():
        return lines

    def update_plot(frame):
        # Update title with live connection status
        state_str = logger.status_message
        if logger.max_rows_reached:
            state_str += f" [CAPPED AT {logger.max_rows} ROWS]"

        if "CONNECTED" in state_str:
            indicator = "🟢"
        elif "PAUSED" in state_str:
            indicator = "🟡"
        else:
            indicator = "🔴"

        ax.set_title(
            f"{indicator} {state_str}\n"
            f"[Space/P]: Pause/Release Port for Upload | Close window to copy CSV to clipboard",
            fontsize=10
        )

        # Pull all pending data points from the thread queue
        updated = False
        while not logger.data_queue.empty():
            try:
                item = logger.data_queue.get_nowait()

                # Check for control event (new run started after upload/reconnect)
                if isinstance(item, dict) and item.get("event") == "NEW_RUN":
                    reset_plot_axes()
                    updated = True
                    continue

                row = item
                t = row[0]
                vals = row[1:]

                # Setup dynamic lines if not initialized yet
                if not lines:
                    num_channels = len(vals)
                    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
                    for i in range(num_channels):
                        c = colors[i % len(colors)]
                        raw_lbl = resolve_label(i, logger.labels)
                        lbl = raw_lbl.replace("_", " ")  # Clean display label for legend
                        (ln,) = ax.plot([], [], label=lbl, color=c, lw=1.6)
                        lines.append(ln)
                        data_bufs.append(deque(maxlen=MAX_PLOT_POINTS))
                    ax.legend(loc="upper right")

                time_buf.append(t)
                for i, v in enumerate(vals):
                    if i < len(data_bufs):
                        data_bufs[i].append(v)
                updated = True
            except queue.Empty:
                break

        if updated and lines and len(time_buf) > 1:
            t_list = list(time_buf)
            for i, ln in enumerate(lines):
                ln.set_data(t_list, list(data_bufs[i]))

            # Dynamic axis scaling
            ax.set_xlim(min(t_list), max(t_list))
            all_vals = [v for buf in data_bufs for v in buf]
            if all_vals:
                min_y, max_y = min(all_vals), max(all_vals)
                pad = max(abs(max_y - min_y) * 0.1, 0.5)
                ax.set_ylim(min_y - pad, max_y + pad)

        return lines

    def on_key(event):
        if event.key in (" ", "p", "P"):
            logger.toggle_pause()

    def on_close(event):
        logger.stop()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", on_close)

    ani = animation.FuncAnimation(
        fig, update_plot, init_func=init_plot, interval=UPDATE_INTERVAL_MS, blit=False
    )

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        logger.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live Arduino Serial Plotter & CSV Logger (Upload-Robust)")
    parser.add_argument("--port", type=str, default=None, help=f"Serial port (default: auto-detected or {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument(
        "--labels",
        nargs="+",
        default=None,
        help="Optional custom labels in order, e.g. --labels Target Actual Ball. Overrides SIGNAL_LABELS list.",
    )
    parser.add_argument(
        "--max-rows",
        type=int,
        default=MAX_LOG_ROWS,
        help=f"Max rows per run to prevent clipboard/memory crash (default: {MAX_LOG_ROWS})",
    )
    args = parser.parse_args()

    chosen_port = args.port if args.port else find_default_port()
    run_plotter(chosen_port, args.baud, labels=args.labels, max_rows=args.max_rows)
