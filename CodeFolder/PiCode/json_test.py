import socket
import json
import time
import errno

ARDUINO_IP = "134.50.51.22"
LISTEN_PORT = 6006
SEND_PORT = 5005
UPDATE_INTERVAL = 10

# Setup Listening Socket
server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind(("0.0.0.0", LISTEN_PORT))
server_sock.listen(5)
server_sock.setblocking(False)

last_send_time = 0

print(f"Monitoring Arduino on port {LISTEN_PORT}...")
print("Press Ctrl+C to stop.")

try:
    while True:
        # --- 1. RECEIVE FROM ARDUINO ---
        try:
            conn, addr = server_sock.accept()
            with conn:
                conn.settimeout(0.5)
                data = conn.recv(1024)
                if data:
                    try:
                        # Clean and Parse
                        decoded_raw = data.decode('utf-8').strip()
                        payload = json.loads(decoded_raw)
                        print(f"Received: {payload}")
                    except json.JSONDecodeError:
                        print("Skipping malformed/partial JSON packet.")
                    except Exception as e:
                        print(f"Error processing data: {e}")
        except socket.error as e:
            # Ignore "No data" errors, report actual network errors
            if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                print(f"Socket error: {e}")

        # --- 2. SEND TO ARDUINO ---
        current_time = time.time()
        if current_time - last_send_time > UPDATE_INTERVAL:
            try:
                # Use a context manager for the outgoing socket too
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(2.0)
                    s.connect((ARDUINO_IP, SEND_PORT))
                    # Example update payload
                    out_payload = {"pi_state": "active", "timestamp": int(current_time)}
                    s.sendall(json.dumps(out_payload).encode('utf-8'))
                    print("Sent update to Arduino.")
            except (socket.timeout, socket.error):
                print("Arduino unreachable, will retry next cycle.")

            last_send_time = current_time

        # CPU relief
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[INFO] Shutdown requested by user.")
finally:
    server_sock.close()
    print("[INFO] Socket closed. Clean exit.")
