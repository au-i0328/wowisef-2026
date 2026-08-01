import sys
import time
import argparse
import pygame
from pygame._sdl2 import controller
import serial
import serial.tools.list_ports

# --- Configuration ---
BAUD_RATE = 115200  # Upgraded to 115200 to prevent serial lag/buffer backup
SEND_RATE = 30      # Update frequency in Hz (1 frame = ~33.3ms)
HOLD_FRAMES = 3    # 10 frames @ 30 Hz = ~333ms pulse duration

def find_arduino_port():
    """Attempts to auto-detect the Arduino serial port on macOS."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device or "usbserial" in port.device:
            return port.device
    return None

def normalize_stick(val):
    return max(0, min(255, int((val + 32768) / 65535.0 * 255)))

def normalize_trigger(val):
    if val < 0:
        val = (val + 32768) / 65535.0 * 255
    else:
        val = (val / 32767.0) * 255
    return max(0, min(255, int(val)))

def get_21_channels(pad):
    """Reads 21 discrete channels using SDL2 standard numerical IDs."""
    
    # Axes
    lx = normalize_stick(pad.get_axis(0))
    ly = normalize_stick(pad.get_axis(1))
    rx = normalize_stick(pad.get_axis(2))
    ry = normalize_stick(pad.get_axis(3))
    l2 = normalize_trigger(pad.get_axis(4))
    r2 = normalize_trigger(pad.get_axis(5))

    # Face & System
    cross    = 1 if pad.get_button(0) else 0
    circle   = 1 if pad.get_button(1) else 0
    square   = 1 if pad.get_button(2) else 0
    triangle = 1 if pad.get_button(3) else 0
    ps_btn   = 1 if pad.get_button(5) else 0

    # D-Pad
    dp_up    = 1 if pad.get_button(11) else 0
    dp_down  = 1 if pad.get_button(12) else 0
    dp_left  = 1 if pad.get_button(13) else 0
    dp_right = 1 if pad.get_button(14) else 0

    # Shoulders, Clicks & Utility
    l1       = 1 if pad.get_button(9) else 0
    r1       = 1 if pad.get_button(10) else 0
    share    = 1 if pad.get_button(4) else 0
    options  = 1 if pad.get_button(6) else 0
    l3       = 1 if pad.get_button(7) else 0
    r3       = 1 if pad.get_button(8) else 0

    return [
        lx, ly, rx, ry, l2, r2,
        cross, circle, square, triangle, ps_btn,
        dp_up, dp_down, dp_left, dp_right,
        l1, r1, share, options, l3, r3
    ]

def main():
    parser = argparse.ArgumentParser(description="SDL2 PS4 Controller Logic Stream")
    parser.add_argument("--test", action="store_true", help="Run without an Arduino connected")
    args = parser.parse_args()

    pygame.init()
    controller.init()

    pad = None
    for i in range(controller.get_count()):
        if controller.is_controller(i):
            pad = controller.Controller(i)
            pad.init()
            break

    if not pad:
        print("❌ No PS4 controller detected! Ensure it is paired/connected and try again.")
        sys.exit(1)

    print(f"🎮 Connected Controller: {pad.name}")

    test_mode = args.test
    ser = None

    if not test_mode:
        port_name = find_arduino_port()
        if not port_name:
            print("⚠️ Could not auto-detect Arduino.")
            user_choice = input("Run in Test Mode without Arduino? (y/n): ").strip().lower()
            if user_choice == 'y':
                test_mode = True
            else:
                port_name = input("Enter custom serial port path: ").strip()

        if not test_mode:
            try:
                ser = serial.Serial(port_name, BAUD_RATE, timeout=1)
                time.sleep(2)
                print(f"🔌 Connected to Arduino on {port_name} @ {BAUD_RATE} baud")
            except Exception as e:
                print(f"❌ Serial connection failed: {e}")
                sys.exit(1)

    if test_mode:
        print("🧪 RUNNING IN TEST MODE (No Arduino connection required)\n")

    clock = pygame.time.Clock()
    print("🚀 Streaming parsed logic to Arduino... Press Ctrl+C to stop.\n")

    # --- State Tracking Variables ---
    drive_direction = "FORWARD" # Default direction on startup
    prev_channels = [0] * 21    # For edge detection
    
    # --- 300ms Pulse Holding Variables ---
    active_command = "NONE"
    hold_counter = 0

    try:
        while True:
            pygame.event.pump()
            channels = get_21_channels(pad)
            
            # Helper for edge detection (returns True only on frame button is pushed down)
            def just_pressed(idx):
                return channels[idx] == 1 and prev_channels[idx] == 0

            # --- 1. Base Direction (L1 / R1) ---
            if channels[15] == 1:
                drive_direction = "FORWARD"
            elif channels[16] == 1:
                drive_direction = "BACKWARD"

            # --- 2. Base Speed (L2) ---
            drive_speed = channels[4]

            # --- 3. D-Pad Overrides (Ignores L2 while held) ---
            if channels[11] == 1:    # D-Pad UP
                drive_speed = 204
                drive_direction = "FORWARD"
            elif channels[12] == 1:  # D-Pad DOWN
                drive_speed = 204
                drive_direction = "BACKWARD"

            # --- 4. Button Command Pulses (Triggers 300ms Hold) ---
            if just_pressed(6):        # Cross
                active_command = "down_detach"
                hold_counter = HOLD_FRAMES
            elif just_pressed(7):      # Circle
                active_command = "down_attach"
                hold_counter = HOLD_FRAMES
            elif just_pressed(8):      # Square
                active_command = "up_attach"
                hold_counter = HOLD_FRAMES
            elif just_pressed(9):      # Triangle
                active_command = "up_detach"
                hold_counter = HOLD_FRAMES
            elif just_pressed(17):     # Share
                active_command = "both_attach"
                hold_counter = HOLD_FRAMES
            elif just_pressed(18):     # Options
                active_command = "both_detach"
                hold_counter = HOLD_FRAMES

            # Manage hold countdown timer
            if hold_counter > 0:
                command_out = active_command
                hold_counter -= 1
            else:
                command_out = "NONE"
                active_command = "NONE"

            # Save state for edge detection in the next loop
            prev_channels = list(channels)

            # --- 5. Construct Payloads & Output ---
            
            # Payload sent to Arduino: Speed,Direction,Command
            serial_payload = f"{drive_speed},{drive_direction},{command_out}\n"
            
            # CSV string for Terminal debugging
            raw_csv = ",".join(map(str, channels))

            if ser:
                ser.write(serial_payload.encode('utf-8'))
            
            # Print live debugging info to terminal
            sys.stdout.write(f"\r\033[K[RAW] {raw_csv}  =>  [SERIAL] {serial_payload.strip()}")
            sys.stdout.flush()

            clock.tick(SEND_RATE)
           
           # Check if Arduino sent an ACK reply
            if ser and ser.in_waiting > 0:
                ack_response = ser.readline().decode('utf-8', errors='ignore').strip()
                if ack_response:
                    print(f"\n[ARDUINO REPLY] {ack_response}")

    except KeyboardInterrupt:
        print("\n\nStopping stream...")
    finally:
        if ser:
            ser.close()
        pygame.quit()
        print("Cleaned up and shut down.")

if __name__ == "__main__":
    main()
