# Smart-Luggage-System

The Smart Luggage System is an innovative solution designed to assist physically challenged individuals by automating luggage transportation. It follows the user autonomously and provides real-time weight measurement, making travel more convenient and effortless.

Features:

✅ User Following Mechanism
Uses ESP32-CAM to detect and follow an ArUco marker attached to the user.
Enables hands-free movement without manual control.


✅ Weight Measurement
Equipped with a load cell and HX711 module to measure luggage weight.
Displays the weight on an I2C display for real-time monitoring.


✅ Travel Assistance
Can carry up to 5kg of weight, making it practical for daily use.
Reduces the need for physical effort in carrying luggage.


Technology Used:
ESP32-CAM → Vision-based user tracking.
Load Cell & HX711 Module → Weight sensing.
I2C Display → Real-time weight display.


Project Impact:
This smart luggage system enhances mobility and independence for physically challenged individuals by providing a hands-free, intelligent carrying solution.


Hardware Requirements:
ESP32-CAM
Load Cell with HX711 module
I2C Display
Power Supply
Chassis and Wheels

Software Requirements:
Arduino IDE
Python (for ArUco marker detection and testing)
