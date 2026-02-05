# Autonomous Line Follower Robot

An autonomous robot project designed to navigate tracks and avoid obstacles using Computer Vision. Built with **Raspberry Pi 5** and **Python**.

## Features
* **Vision System:** Uses Raspberry Pi Camera Module 3.
* **Image Processing:** Converts RGB to **HSV** for robust line detection.
* **Control Algorithm:** Implements a **PI Controller** for smooth steering.
* **Obstacle Avoidance:** Detects obstacles using Color Thresholding algorithms.
* **Motor Control:** Precise speed management via PWM signals.

## Hardware Stack
* Raspberry Pi 5
* Camera Module 3 (Wide Angle)
* L298N Motor Driver
* DC Motors & Custom Chassis

## Software Stack
* Python 3.x
* OpenCV (cv2)
* NumPy
* GPIO Zero

## Challenges Solved
Originally planned to use YOLOv8 for object detection. However, due to the camera's Field of View and the close proximity of objects, I optimized the system using **Color Thresholding**, which proved to be faster and more reliable for this specific chassis design.
