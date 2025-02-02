import cv2
import numpy as np
import urllib.request

# URL for the camera feed
url = 'http://192.168.208.21/cam-hi.jpg'  # Replace with your ESP32-CAM's URL

# Main loop to read and display the video stream
while True:
    # Capture image from URL
    img_resp = urllib.request.urlopen(url)
    img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    img = cv2.imdecode(img_np, -1)

    # Display the captured image
    cv2.imshow('ESP32-CAM Feed', img)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
