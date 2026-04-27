import cv2
import socket
import struct
import pickle
import numpy as np

LAPTOP_IP = '0.0.0.0'
PORT = 9999

def start_viewer():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((LAPTOP_IP, PORT))
    server_socket.listen(5)
    print(f"[System] Viewer active on port {PORT}. Calibration parameters loaded.")

    window_name = "Go2 Inspection - Calibrated 3D Feed"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)

    while True:
        conn, addr = server_socket.accept()
        data = b""
        payload_size = struct.calcsize("Q")
        
        try:
            while True:
                while len(data) < payload_size:
                    packet = conn.recv(4096)
                    if not packet: break
                    data += packet
                
                if not data: break
                
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]
                
                while len(data) < msg_size:
                    data += conn.recv(4096)
                
                frame_data = data[:msg_size]
                data = data[msg_size:]
                
                payload = pickle.loads(frame_data)
                if isinstance(payload, dict):
                    frame_buffer = payload.get("jpeg")
                else:
                    frame_buffer = payload
                frame = cv2.imdecode(frame_buffer, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    cv2.imshow("Go2 Inspection - Calibrated 3D Feed", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            print(f"[Error] Connection error: {e}")
        finally:
            conn.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    start_viewer()
