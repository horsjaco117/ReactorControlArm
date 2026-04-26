import curses
import serial
import threading
import time
import random
from collections import deque

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  # Change to /dev/ttyUSB0 if needed
BAUD_RATE = 115200

# Global State Dictionary to hold the live bits
sys_state = {
    'scram': False, 'power': False, 'magnet': False,
    'forward': False, 'backward': False, 'pos_min': False,
    'pos_max': False, 'speed': False, 'control': False,
    'pos_set': 0, 'pos_read': 0, 'knob1': 0, 'knob2': 0,
    'connected': False
}

# UI Globals
current_screen = "MAIN"  # Can be "MAIN" or "HELP"
user_input = ""
last_glitch_time = time.time()

def serial_worker(port):
    """Background thread to hunt for telemetry packets and read text."""
    global sys_state
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        sys_state['connected'] = True
    except:
        sys_state['connected'] = False
        return

    # A sliding window to catch the 11-byte binary packet ending in 0x24
    window = deque(maxlen=11)

    while True:
        try:
            if ser.in_waiting > 0:
                raw_bytes = ser.read(ser.in_waiting)
                for byte in raw_bytes:
                    window.append(byte)
                    
                    # If we have 11 bytes and the last byte is our Sync Byte (0x24)
                    if len(window) == 11 and window[10] == 0x24:
                        # Extract Packets
                        p1 = window[0]
                        p2 = window[1]
                        
                        # Unpack bits
                        sys_state['scram']    = bool(p1 & (1 << 0))
                        sys_state['power']    = bool(p1 & (1 << 1))
                        sys_state['magnet']   = bool(p1 & (1 << 2))
                        sys_state['forward']  = bool(p1 & (1 << 3))
                        sys_state['backward'] = bool(p1 & (1 << 4))
                        sys_state['pos_min']  = bool(p1 & (1 << 5))
                        sys_state['pos_max']  = bool(p1 & (1 << 6))
                        sys_state['speed']    = bool(p1 & (1 << 7))
                        sys_state['control']  = bool(p2 & (1 << 0))

                        # Unpack Analog (High Byte << 8 | Low Byte)
                        sys_state['pos_set']  = (window[2] << 8) | window[3]
                        sys_state['pos_read'] = (window[4] << 8) | window[5]
                        sys_state['knob1']    = (window[6] << 8) | window[7]
                        sys_state['knob2']    = (window[8] << 8) | window[9]
        except:
            sys_state['connected'] = False
            time.sleep(1)

def draw_scanlines(stdscr, h, w, amber_dim):
    """Draws alternating characters to simulate CRT scanlines."""
    for y in range(1, h - 3, 2):
        try:
            # Changed from "-" to "=" for a thicker, heavier scanline effect.
            # You can also try "≡" or "═" for different CRT shadow-mask aesthetics.
            stdscr.addstr(y, 0, "=" * (w - 1), amber_dim)
        except curses.error:
            pass

def draw_main_screen(stdscr, amber, h, w):
    """Draws the live bit status grid, dynamically centered based on window width."""
    
    # Calculate column starts based on actual screen width (w)
    col1 = max(4, w // 10)         # 10% from the left
    col2 = max(w // 3, 30)         # 33% from the left
    col3 = max((w // 3) * 2, 60)   # 66% from the left
    
    # Header
    try:
        stdscr.addstr(3, 4, "SYSTEM TELEMETRY LINK:", amber | curses.A_BOLD)
        stdscr.addstr(3, 30, "ONLINE" if sys_state['connected'] else "OFFLINE", amber | curses.A_BLINK)

        # Column 1
        stdscr.addstr(8,  col1, f"SCRAM:    [{'ON ' if sys_state['scram'] else 'OFF'}]", amber)
        stdscr.addstr(12, col1, f"FORWARD:  [{'ON ' if sys_state['forward'] else 'OFF'}]", amber)
        stdscr.addstr(16, col1, f"POS MIN:  [{'ON ' if sys_state['pos_min'] else 'OFF'}]", amber)
        stdscr.addstr(20, col1, f"CONTROL:  [{'RUN' if sys_state['control'] else 'STP'}]", amber)

        # Column 2
        stdscr.addstr(8,  col2, f"POWER:    [{'ON ' if sys_state['power'] else 'OFF'}]", amber)
        stdscr.addstr(12, col2, f"BACKWARD: [{'ON ' if sys_state['backward'] else 'OFF'}]", amber)
        stdscr.addstr(16, col2, f"POS MAX:  [{'ON ' if sys_state['pos_max'] else 'OFF'}]", amber)
        stdscr.addstr(20, col2, f"SPEED:    [{'FST' if sys_state['speed'] else 'SLW'}]", amber)

        # Column 3
        stdscr.addstr(8,  col3, f"MAGNET:   [{'ON ' if sys_state['magnet'] else 'OFF'}]", amber)
        stdscr.addstr(12, col3, f"POS SET:  {sys_state['pos_set']:04d}", amber)
        stdscr.addstr(16, col3, f"POS READ: {sys_state['pos_read']:04d}", amber)
        stdscr.addstr(20, col3, f"KNOBS:    {sys_state['knob1']:04d} | {sys_state['knob2']:04d}", amber)
    except curses.error:
        # If the window is too small even for this, do nothing to prevent the crash
        pass
def draw_help_screen(stdscr, amber, h, w):
    """Draws the static help documentation."""
    stdscr.addstr(2, 2, "HORSCO OS - COMMAND MANUAL", amber | curses.A_BOLD)
    
    commands = [
        "COMMAND        ALIAS     FUNCTION",
        "--------------------------------------------------",
        "scram          s         Toggle emergency SCRAM",
        "power          p         Toggle main system power",
        "magnet         m         Toggle electromagnet lock",
        "forward        f         Engage forward drive",
        "backward       b         Engage backward drive",
        "min / max                Toggle positional overrides",
        "speed          sp        Toggle motor step timing",
        "fullscreen     F11       Press F11 for nice display"
        "--------------------------------------------------",
        "Type 'main' to return to system telemetry."
    ]
    
    for i, line in enumerate(commands):
        stdscr.addstr(5 + i, 4, line, amber)

def boot_sequence(stdscr, amber, h, w):
    """Simulates a RobCo/Fallout BIOS boot sequence."""
    stdscr.clear()
    
    boot_text = [
        "HORSCO INDUSTRIES UNIFIED OPERATING SYSTEM",
        "COPYRIGHT 2075-2077 HORSCO INDUSTRIES",
        "-Server 1-",
        "",
        "Initializing HORSCO Telemetry Uplink..."
    ]
    
    # Print the initial header slowly
    for i, line in enumerate(boot_text):
        try:
            stdscr.addstr(2 + i, 2, line, amber | curses.A_BOLD)
            stdscr.refresh()
            time.sleep(0.4)
        except curses.error:
            pass
            
    # Simulate a retro memory check
    try:
        stdscr.addstr(8, 2, "640K RAM SYSTEM", amber)
        stdscr.refresh()
        time.sleep(0.3)
        stdscr.addstr(9, 2, "38911 BYTES FREE", amber)
        stdscr.refresh()
        time.sleep(0.8)
        
        # Simulate hardware connection wait
        stdscr.addstr(11, 2, "Attempting USB handshake with mainframe...", amber)
        stdscr.refresh()
        time.sleep(1.0)
        
        # Draw a fake, jittery loading bar
        stdscr.addstr(13, 2, "UPLINK: [", amber)
        for i in range(25):
            stdscr.addstr(13, 11 + i, "=", amber)
            stdscr.refresh()
            # Random pauses make it feel like struggling hardware
            time.sleep(random.uniform(0.01, 0.15)) 
            
        stdscr.addstr(13, 36, "] ESTABLISHED", amber | curses.A_BLINK)
        stdscr.refresh()
        time.sleep(1.5)
    except curses.error:
        pass

    stdscr.clear()

def main_tui(stdscr):
    global current_screen, user_input, last_glitch_time

    # Curses setup
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_YELLOW, -1)  # Amber text
    curses.init_pair(2, curses.COLOR_YELLOW, -1)  # Dim amber
    curses.curs_set(0) # Hide real cursor
    stdscr.nodelay(True) # Non-blocking keyboard input

    amber = curses.color_pair(1)
    amber_dim = curses.color_pair(2) | curses.A_DIM

    # Get initial screen dimensions
    h, w = stdscr.getmaxyx()

    # --- RUN BOOT SEQUENCE HERE ---
    boot_sequence(stdscr, amber, h, w)

    # Start Serial Thread
    threading.Thread(target=serial_worker, args=(SERIAL_PORT,), daemon=True).start()

    # Hardware connection object for writing commands
    try:
        ser_write = serial.Serial(SERIAL_PORT, BAUD_RATE)
    except:
        ser_write = None

    while True:
        h, w = stdscr.getmaxyx()
        stdscr.erase()

        # 1. Fake CRT Glitch Generator (1% chance per frame)
        is_glitching = False
        if random.random() > 0.99 and time.time() - last_glitch_time > 2.0:
            is_glitching = True
            last_glitch_time = time.time()

        # 2. Draw CRT Scanlines (Background)
        draw_scanlines(stdscr, h, w, amber_dim)

        # 3. Top Header
        header = " HORSCO INDUSTRIES UNIFIED OPERATING SYSTEM v8.4 "
        # If glitching, shift the header slightly
        x_offset = 2 if not is_glitching else random.randint(0, 5)
        stdscr.addstr(0, x_offset, header, amber | curses.A_REVERSE)

        # 4. Render Active Screen
        if current_screen == "MAIN":
            draw_main_screen(stdscr, amber, h, w)
        elif current_screen == "HELP":
            draw_help_screen(stdscr, amber, h, w)

        # 5. Fixed Command Bar at Bottom
        stdscr.addstr(h - 3, 0, "=" * (w - 1), amber)
        
        prompt = f"UPLINK> {user_input}"
        # Cursor blink effect
        if int(time.time() * 2) % 2 == 0:
            prompt += "_"
        
        # If glitching, dim the command bar briefly
        cmd_color = amber if not is_glitching else amber_dim
        stdscr.addstr(h - 2, 2, prompt, cmd_color)

        # 6. Handle Keyboard Input
        try:
            c = stdscr.getch()
            if c != -1:
                # --- NEW ESCAPE KEY LOGIC ---
                if c == 27:  # 27 is the ASCII code for the Escape key
                    break    # Breaks the loop and cleanly exits the program
                # ----------------------------
                
                elif c in (curses.KEY_BACKSPACE, 8, 127):
                    user_input = user_input[:-1]
                elif c in (curses.KEY_ENTER, 10, 13):
                    cmd = user_input.strip().lower()
                    user_input = "" # Clear input
                    
                    # Local Screen Routing
                    if cmd in ['help', '?']:
                        current_screen = "HELP"
                    elif cmd in ['main', 'exit', 'quit']:
                        current_screen = "MAIN"
                    elif cmd == 'kill':
                        break # Exits the Python script
                    else:
                        # Send to Arduino
                        if ser_write and ser_write.is_open and cmd:
                            ser_write.write((cmd + '\n').encode('utf-8'))
                            
                # Allow standard typing
                elif 32 <= c <= 126:
                    user_input += chr(c)
        except curses.error:
            pass

if __name__ == "__main__":
    try:
        curses.wrapper(main_tui)
    except KeyboardInterrupt:
        print("\nTerminal connection closed.")
