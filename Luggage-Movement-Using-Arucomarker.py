import cv2
import numpy as np
import math
import urllib.request
import socket
import time

# Configuration for source type ("esp32" or "webcam")
source = "esp32"  # Change to "webcam" to use the computer camera

# URL for ESP32-CAM feed
esp32_url = 'http://192.168.151.21/cam-hi.jpg'  # Replace with your ESP32-CAM's URL

# Function to calculate distance between two points
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Function to determine movement command with proportional control
def robo_command(marker_center, frame_center, distance, stop_threshold=50, angle_threshold=50):
    x_diff = marker_center[0] - frame_center[0]
    distance_factor = max(0.5, 1 - (distance / 500))  # Scale factor for speed adjustment
    
    if distance < stop_threshold:
        return "stop", 0  # Stop movement when close
    
    if abs(x_diff) <= angle_threshold:
        return "forward", distance_factor  # Move forward with proportional speed
    
    turn_speed = min(1.0, abs(x_diff / frame_center[0]))  # Turn proportional to deviation
    if x_diff > angle_threshold:
        return "left", turn_speed  # Adjust speed based on deviation
    elif x_diff < -angle_threshold:
        return "right", turn_speed  # Adjust speed based on deviation

# Function to detect and process ArUco markers
def detect_and_process_aruco(frame):
    frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        marker_center = np.mean(corners[0][0], axis=0).astype(int)
        distance = calculate_distance(frame_center, marker_center)
        command, speed = robo_command(marker_center, frame_center, distance)

        # Draw marker and information on the frame
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
    ip, port = "192.168.151.210", 8002
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
                conn.sendall(str.encode(f"{command}\n"))
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