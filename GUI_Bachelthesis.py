import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports

class ArduinoValueGUI:
    def __init__(self, master):
        self.master = master
        master.title("Joystick Values & Mode")

        # Serial object (initialized on Connect)
        self.ser = None
        self._reading = False

        # =================================================================================
        # 1) Serial Port Selection
        # =================================================================================
        port_frame = ttk.LabelFrame(master, text="Serial Port")
        port_frame.grid(row=0, column=0, columnspan=4, padx=10, pady=5, sticky="ew")

        self.port_var = tk.StringVar()
        self.ports = self._detect_serial_ports()
        self.port_menu = ttk.Combobox(
            port_frame,
            textvariable=self.port_var,
            values=self.ports,
            width=25,
            state="readonly"
        )
        self.port_menu.grid(row=0, column=0, padx=5, pady=5)
        if self.ports:
            self.port_var.set(self.ports[0])

        ttk.Label(port_frame, text="Baud Rate:").grid(row=0, column=1, padx=(10,2))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_entry = ttk.Entry(port_frame, textvariable=self.baud_var, width=8)
        self.baud_entry.grid(row=0, column=2, padx=2, pady=5)

        self.connect_btn = ttk.Button(port_frame, text="Connect", command=self._connect_serial)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=5)
        self.disconnect_btn = ttk.Button(port_frame, text="Disconnect", command=self._disconnect_serial, state="disabled")
        self.disconnect_btn.grid(row=0, column=4, padx=5, pady=5)

        # =================================================================================
        # 2) Setup Buttons for Calibration (Start / Skip)
        # =================================================================================
        setup_frame = ttk.LabelFrame(master, text="Setup")
        setup_frame.grid(row=1, column=0, columnspan=4, padx=10, pady=5, sticky="ew")

        self.start_btn = ttk.Button(setup_frame, text="Start Calibration", command=self._send_start)
        self.start_btn.grid(row=0, column=0, padx=10, pady=5)

        self.skip_btn = ttk.Button(setup_frame, text="Skip Calibration", command=self._send_skip)
        self.skip_btn.grid(row=0, column=1, padx=10, pady=5)

        # =================================================================================
        # 3) Display of 5 Live Values (left) + Two Images (right)
        # =================================================================================
        values_frame = ttk.LabelFrame(master, text="Live Values")
        values_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")

        labels = ["Pitch (°)", "Roll (°)", "Ll (mm)", "Lr (mm)", "rawX (Joystick)"]
        self.value_vars = [tk.StringVar(value="–") for _ in labels]
        for i, txt in enumerate(labels):
            ttk.Label(values_frame, text=txt + ":").grid(row=i, column=0, sticky="e", padx=5, pady=2)
            ttk.Label(
                values_frame,
                textvariable=self.value_vars[i],
                width=12,
                relief="sunken"
            ).grid(row=i, column=1, sticky="w", padx=5, pady=2)

        # ==== Image 1: “Exploded view_double parallelogram mechanism.png” ====
        try:
            img1 = Image.open("Complete design.jpg")
            img1 = img1.resize((350, 300), Image.LANCZOS)
            self.logo_img1 = ImageTk.PhotoImage(img1)
        except Exception as e:
            messagebox.showwarning(
                "Image Error",
                "Could not load 'Exploded view_double parallelogram mechanism.png':\n" + str(e)
            )
            self.logo_img1 = None

        if self.logo_img1:
            logo_label1 = ttk.Label(master, image=self.logo_img1)
            logo_label1.grid(row=2, column=2, padx=10, pady=5, sticky="n")

        # ==== Image 2: “Observation2_Roll_Angle.png” ====
        try:
            img2 = Image.open("Observation2_Roll_Angle.png")
            img2 = img2.resize((400, 300), Image.LANCZOS)
            self.logo_img2 = ImageTk.PhotoImage(img2)
        except Exception as e:
            messagebox.showwarning(
                "Image Error",
                "Could not load 'Observation2_Roll_Angle.png':\n" + str(e)
            )
            self.logo_img2 = None

        if self.logo_img2:
            logo_label2 = ttk.Label(master, image=self.logo_img2)
            logo_label2.grid(row=2, column=3, padx=10, pady=5, sticky="n")

        # =================================================================================
        # 4) Control Mode Buttons
        # =================================================================================
        mode_frame = ttk.LabelFrame(master, text="Control Mode")
        mode_frame.grid(row=3, column=0, columnspan=4, padx=10, pady=5, sticky="ew")

        self.abs_btn = ttk.Button(mode_frame, text="Absolute (A)", command=self._send_mode_absolute)
        self.abs_btn.grid(row=0, column=0, padx=10, pady=5)

        self.inc_btn = ttk.Button(mode_frame, text="Incremental (I)", command=self._send_mode_incremental)
        self.inc_btn.grid(row=0, column=1, padx=10, pady=5)

        # =================================================================================
        # 5) Reset Button (for resetting Arduino during runtime)
        # =================================================================================
        reset_frame = ttk.LabelFrame(master, text="Reset")
        reset_frame.grid(row=4, column=0, columnspan=4, padx=10, pady=5, sticky="ew")

        self.reset_btn = ttk.Button(reset_frame, text="Reset Arduino", command=self._send_reset)
        self.reset_btn.grid(row=0, column=0, padx=10, pady=5)

        # =================================================================================
        # 6) Status Textbox
        # =================================================================================
        status_frame = ttk.LabelFrame(master, text="Status")
        status_frame.grid(row=5, column=0, columnspan=4, padx=10, pady=(5,10), sticky="ew")
        self.status_text = tk.Text(status_frame, height=7, state="disabled")
        self.status_text.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # =================================================================================
        # Grid Configuration so that columns stretch appropriately
        # =================================================================================
        master.columnconfigure(0, weight=1)
        master.columnconfigure(1, weight=1)
        master.columnconfigure(2, weight=0)  # Fixed width for Image 1
        master.columnconfigure(3, weight=0)  # Fixed width for Image 2
        values_frame.columnconfigure(1, weight=1)
        status_frame.columnconfigure(0, weight=1)

    def _detect_serial_ports(self):
        """Return a list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def _connect_serial(self):
        """Open the selected COM port with the given baud rate."""
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid baud rate")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            messagebox.showerror("Error", f"Cannot open {port}:\n{e}")
            return

        self._log(f"Connected to {port} @ {baud} baud")
        self.connect_btn.config(state="disabled")
        self.disconnect_btn.config(state="normal")
        self._reading = True
        self.master.after(50, self._read_serial)

    def _disconnect_serial(self):
        """Close the serial connection."""
        self._reading = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self._log("Serial connection closed")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        # Reset displayed values
        for var in self.value_vars:
            var.set("–")

    def _read_serial(self):
        """Called every 50 ms: read one line, update live values separately, and log others."""
        if not self._reading or self.ser is None or not self.ser.is_open:
            return

        try:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
        except Exception as e:
            self._log(f"Read error: {e}")
            self.master.after(50, self._read_serial)
            return

        if line:
            parts = line.split('\t')
            if len(parts) == 5:
                # Parsen der fünf numerischen Werte, aber nur in die Live-Value-Felder schreiben,
                # nicht in die Status-Textbox.
                try:
                    pitch = float(parts[0])
                    roll  = float(parts[1])
                    ll    = float(parts[2])
                    lr    = float(parts[3])
                    rawx  = int(float(parts[4]))
                    # Live-Werte aktualisieren:
                    self.value_vars[0].set(f"{pitch:.2f}")
                    self.value_vars[1].set(f"{roll:.2f}")
                    self.value_vars[2].set(f"{ll:.2f}")
                    self.value_vars[3].set(f"{lr:.2f}")
                    self.value_vars[4].set(f"{rawx:d}")
                    # Nicht in den Statuslog schreiben!
                except ValueError:
                    # Wenn Parsing fehlschlägt, diese Zeile als nicht fünf Werte behandeln:
                    self._log(f"Received malformed numeric line: {line}")
            else:
                # Alle anderen Zeilen (Mode-/Kalibrierungstexte etc.) in Status-Textbox loggen:
                self._log(line)

        # Nächsten Aufruf in 50 ms planen
        self.master.after(50, self._read_serial)

    def _send_start(self):
        """
        Send the text 'Start\n' to Arduino to begin stepper calibration.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"Start\n")
                self._log("→ 'Start' sent (Begin Calibration)")
            except Exception as e:
                self._log(f"Error sending 'Start': {e}")

    def _send_skip(self):
        """
        Send the text 'Skip\n' to Arduino to skip stepper calibration.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"Skip\n")
                self._log("→ 'Skip' sent (Skip Calibration)")
            except Exception as e:
                self._log(f"Error sending 'Skip': {e}")

    def _send_mode_absolute(self):
        """Send the character 'a' to Arduino to activate absolute mode."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'a')
                self._log("→ 'a' sent (Absolute mode)")
            except Exception as e:
                self._log(f"Error sending 'a': {e}")

    def _send_mode_incremental(self):
        """Send the character 'i' to Arduino to activate incremental mode."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'i')
                self._log("→ 'i' sent (Incremental mode)")
            except Exception as e:
                self._log(f"Error sending 'i': {e}")

    def _send_reset(self):
        """
        Send the character 'r' to Arduino to trigger a reset (esp_restart()).
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'r')
                self._log("→ 'r' sent (Reset Arduino)")
            except Exception as e:
                self._log(f"Error sending 'r': {e}")

    def _log(self, text):
        """Write a message to the status textbox."""
        self.status_text.config(state="normal")
        self.status_text.insert("end", text + "\n")
        self.status_text.see("end")
        self.status_text.config(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoValueGUI(root)
    root.mainloop()
