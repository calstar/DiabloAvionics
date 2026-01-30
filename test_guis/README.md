# test_guis

Combined GUI for actuator control and PT data plotting, matching the behavior of the standalone actuator control GUI and PT_BOARD_Multi GUI side by side.

## combined_actuator_pt_gui.py

- **Left panel:** Actuator control (same as `actuator_control_gui.py` / ADC_Testing/Actuator_Testing). Send actuator commands to the actuator board and display current-sense voltage readings.
- **Right panel:** Live PT voltage plots (same as PT_BOARD_Multi_Gui). Receive DAQv2-Comms SENSOR_DATA from the PT board and plot PT 1–10 over time.

### Firmware

Use the same firmware as before; only IP/port need to be set per board:

- **Actuator board** (ADC_Testing/Actuator_Testing):  
  - Listen for commands on UDP port **5005**.  
  - Send sensor data to this PC’s IP on port **5006** (default).

- **PT board** (PT_BOARD_Multi_Send):  
  - Send SENSOR_DATA to this PC’s IP on port **5007** (default).  
  - Set `receiverPort` to **5007** in firmware so it does not conflict with actuator data on 5006.

### Run

```bash
pip install -r requirements.txt
python combined_actuator_pt_gui.py
```

Optional arguments:

- `-i` / `--actuator-ip` — Actuator board IP (default `192.168.2.100`)
- `--actuator-port` — Actuator command port (default `5005`)
- `--actuator-receive-port` — Port for actuator sensor data (default `5006`)
- `--pt-port` — Port for PT sensor data (default `5007`)
- `-a` / `--address` — Bind address (default `0.0.0.0`)

Use **Settings** in the GUI to change IPs and ports at runtime.
