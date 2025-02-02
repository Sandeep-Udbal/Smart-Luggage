import cv2
import numpy as np
import math
import urllib.request
import socket
import time

# Configuration for source type ("esp32" or "webcam")
source = "esp32"  # Change to "webcam" to use the computer camera

# URL for ESP32-CAM feed
esp32_url = 'http://192.168.208.21/cam-hi.jpg'  # Replace with your ESP32-CAM's URL

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# PID controller for x-axis error (turning)
pid = PIDController(kp=0.5, ki=0.1, kd=0.05)

# Function to calculate distance between two points
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Function to calculate angular error
def calculate_angle_error(marker_center, frame_center):
    dx = marker_center[0] - frame_center[0]
    dy = frame_center[1] - marker_center[1]  # Assuming y-axis increases downwards
    angle = math.degrees(math.atan2(dy, dx))
    return angle

# Function to detect and process ArUco markers
def detect_and_process_aruco(frame):
    frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        # Select the marker closest to the center
        distances = [calculate_distance(frame_center, np.mean(corners[i][0], axis=0)) for i in range(len(ids))]
        closest_idx = np.argmin(distances)
        marker_center = np.mean(corners[closest_idx][0], axis=0).astype(int)
        distance = distances[closest_idx]
        angle_error = calculate_angle_error(marker_center, frame_center)
        x_error = marker_center[0] - frame_center[0]

        # PID-based turn speed
        turn_speed = pid.compute(x_error)

        # Determine command
        if distance < 50:
            command = "stop"
            speed = 0
        elif abs(x_error) < 10:
            command = "forward"
            speed = max(0.3, min(1.0, distance / 500))
        else:
            command = "right" if x_error > 0 else "left"
            speed = abs(turn_speed)

        # Draw marker and information
        cv2.circle(frame, tuple(marker_center), 5, (0, 255, 0), -1)
        cv2.line(frame, frame_center, tuple(marker_center), (255, 0, 0), 2)
        cv2.putText(frame, f"Distance: {distance:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Command: {command}, Speed: {speed:.2f}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return frame, command, speed
    return frame, "stop", 0

# Function to create socket connection
def create_socket_connection(ip, port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((ip, port))
        s.listen()
        conn, addr = s.accept()
        print(f'Connected to: {addr}')
        return s, conn
    except Exception as e:
        print(f"Error creating socket connection: {e}")
        return None, None

if __name__ == "__main__":
    print("Starting ArUco Marker Follower")

    # Camera feed setup
    if source == "webcam":
        cap = cv2.VideoCapture(0)

    # Socket setup
    ip, port = "192.168.208.63", 8002
    s, conn = create_socket_connection(ip, port)
    if s is None or conn is None:
        exit(1)

    while True:
        try:
            if source == "esp32":
                try:
                    img_resp = urllib.request.urlopen(esp32_url, timeout=5)
                    img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
                    frame = cv2.imdecode(img_np, -1)
                except Exception as e:
                    print(f"Error accessing ESP32-CAM: {e}")
                    break
            elif source == "webcam":
                ret, frame = cap.read()
                if not ret:
                    print("Error accessing webcam.")
                    break

            # Process frame
            marked_image, command, speed = detect_and_process_aruco(frame)

            # Send command
            try:
                conn.sendall(f"{command}\n")
                print(f"Sent command: {command}")
            except Exception as e:
                print(f"Socket error: {e}")
                s, conn = create_socket_connection(ip, port)

            # Display the feed
            cv2.imshow("Camera Feed", marked_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Exception as e:
            print(f"Runtime error: {e}")
            break

    # Cleanup
    if source == "webcam":
        cap.release()
    if s:
        s.close()
    cv2.destroyAllWindows()