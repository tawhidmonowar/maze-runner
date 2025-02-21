## Maze Runner (Alpha)

![Static Badge](https://img.shields.io/badge/Arduino-%23009baa)
![Static Badge](https://img.shields.io/badge/Line%20Follower%20Robot-8A2BE2)
![Static Badge](https://img.shields.io/badge/PID%20Controller-greenbright)

Maze Runner is a real-time autonomous line-following robot designed for precise path tracking and navigation. It utilizes an Arduino UNO, a Digital IR Sensor Array, and a PID-based control system to efficiently follow lines and adapt to different track conditions.

## Video Preview
https://github.com/user-attachments/assets/a39be2a8-71a3-485b-b143-e1acad353426

## Features
- PID-Based Line Tracking: Implements PID Controller algorithm to enhance accuracy.
- PWM Motor Control: Uses an L293D Motor Driver Shield for smooth and efficient movement.
- Adaptive Sensor Calibration: Optimized IR sensor noise filtering for better track adaptability.
- Competition-Tested: Successfully participated in multiple competitions, earning recognition for innovation and performance.

## Hardware Components
- Arduino UNO – Microcontroller for processing sensor inputs and motor control.
- L293D Motor Driver Shield – Provides bidirectional control for the motors.
- Digital IR Sensor Array – Detects track lines and feeds real-time data for navigation.

## Technologies Used
- Arduino (C++) for firmware development.
- PID Controller for optimized line tracking.
- PWM Motor Control for smooth speed adjustments.

## Images  
| ![image](https://github.com/tawhidmonowar/maze-runner/blob/main/image/1668615205751.jpg) | ![image](https://github.com/user-attachments/assets/4f6b310e-5dba-4060-9ed3-250fea57dcca) |
|---|---|

## How It Works
- The IR sensor array detects the track and sends data to the Arduino UNO.
- The PID algorithm processes the sensor data and calculates adjustments for motor speed.
- The L293D Motor Driver Shield controls the motors using PWM signals, allowing the robot to follow the line smoothly.

## Future Improvements  
- Implementing **machine learning** for adaptive path prediction.  
- Enhancing **obstacle detection** for more complex navigation.
- Integrating **computer vision** for advanced path recognition and decision-making.  
- Developing **maze-solving algorithms** for autonomous navigation in complex environments. 

## License  
This project is open-source under the **MIT License**.  

