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
server_sock.setblocking(False)          # keep non-blocking accept

print(f"Monitoring Arduino on port {LISTEN_PORT}...")
print("Press Ctrl+C to stop.")

last_send_time = 0

try:
    while True:
        # --- 1. RECEIVE FROM ARDUINO (ROBUST FULL-READ) ---
        try:
            conn, addr = server_sock.accept()
            with conn:
                conn.settimeout(2.0)          # generous timeout per connection
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:             # client closed connection (Arduino did stop())
                        break
                    data += chunk
                    if len(data) > 8192:      # safety limit
                        break

                if data:
                    decoded_raw = data.decode('utf-8').strip()
                    try:
                        payload = json.loads(decoded_raw)
                        print(f"✅ Received valid JSON from {addr}: {payload}")
                    except json.JSONDecodeError as e:
                        print(f"❌ Malformed JSON received: {decoded_raw[:300]}...")
                        print(f"   Error: {e}")
                    except Exception as e:
                        print(f"Error processing data: {e}")

        except socket.error as e:
            # Ignore "no connection ready" errors (normal with non-blocking)
            if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN, errno.EINTR):
                print(f"Socket error: {e}")

        # --- 2. SEND TO ARDUINO (unchanged) ---
        current_time = time.time()
        if current_time - last_send_time > UPDATE_INTERVAL:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(2.0)
                    s.connect((ARDUINO_IP, SEND_PORT))
                    out_payload = {"pi_state": "active", "timestamp": int(current_time)}
                    s.sendall(json.dumps(out_payload).encode('utf-8'))
                    print("Sent update to Arduino.")
            except (socket.timeout, socket.error):
                print("Arduino unreachable, will retry next cycle.")
            last_send_time = current_time

        time.sleep(0.05)   # reduced CPU usage

except KeyboardInterrupt:
    print("\n[INFO] Shutdown requested by user.")
finally:
    server_sock.close()
    print("[INFO] Socket closed. Clean exit.")