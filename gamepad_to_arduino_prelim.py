import sys
import time
import argparse
import pygame
from pygame._sdl2 import controller
import serial
import serial.tools.list_ports

# --- Configuration ---
BAUD_RATE = 9600
SEND_RATE = 30  # Update frequency (Hz)

def find_arduino_port():
    """Attempts to auto-detect the Arduino serial port on macOS."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device or "usbserial" in port.device:
            return port.device
    return None

def normalize_stick(val):
    """Normalizes SDL stick values (-32768 to 32767) to 0-255 range."""
    return max(0, min(255, int((val + 32768) / 65535.0 * 255)))

def normalize_trigger(val):
    """Normalizes SDL trigger values to 0-255 range."""
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
    parser = argparse.ArgumentParser(description="SDL2 PS4 Controller Stream (21 Channels)")
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
                print(f"🔌 Connected to Arduino on {port_name}")
            except Exception as e:
                print(f"❌ Serial connection failed: {e}")
                sys.exit(1)

    if test_mode:
        print("🧪 RUNNING IN TEST MODE (No Arduino connection required)\n")

    clock = pygame.time.Clock()
    print("🚀 Streaming 21 unique channels... Press Ctrl+C to stop.\n")

    try:
        while True:
            pygame.event.pump()

            channels = get_21_channels(pad)
            payload = ",".join(map(str, channels)) + "\n"

            if ser:
                ser.write(payload.encode('utf-8'))
                print(f"[SERIAL OUT] {payload.strip()}", end="\r")
            else:
                print(f"[TEST MODE]  {payload.strip()}", end="\r")

            clock.tick(SEND_RATE)

    except KeyboardInterrupt:
        print("\n\nStopping stream...")
    finally:
        if ser:
            ser.close()
        pygame.quit()
        print("Cleaned up and shut down.")

if __name__ == "__main__":
    main()
