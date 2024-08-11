# Problem 1: Angle Recording using MPU-6050

## Introduction to Motion Sensors
Motion sensors are integral components of robotic systems, playing a vital role in tasks such as calibrating and controlling robotic arms through closed-loop mechanisms. These sensors are increasingly utilized in the field of rehabilitation robotics as well. Gyro sensors, also referred to as angular rate or angular velocity sensors are MEMS-based devices designed to measure the angular velocity of moving objects. In the market, there exists a variety of gyro sensors, including models like MPU-6050, MPU9250, and GY-25. These sensors differ in precision, number of measurement axes, and additional features. The MPU-6050 stands out as a popular six-degree-of-freedom (6-DoF) inertial measurement unit (IMU) comprising a 3-axis accelerometer and a 3-axis gyroscope. Leveraging Micro Electro Mechanical Systems (MEMS) technology, it delivers precise measurements of acceleration, velocity, and orientation. The MPU-6050 offers multiple full-scale ranges for both the accelerometer (±2g, ±4g, ±8g, ±16g) and the gyroscope (+/- 250°/s, +/- 500°/s, +/- 1000°/s, +/- 2000°/s). Equipped with a digital motion processor (DMP), this sensor can execute advanced signal processing tasks, making it suitable for diverse applications like smartphones, tablets, drones, robots, and gaming systems. Compact in size at approximately 4 mm X 4 mm X 0.9 mm, the MPU-6050 operates on a 5V supply with low power consumption—drawing less than 3.6 mA during measurements and only 5 µA when idle. Its energy efficiency enables integration into battery-operated devices. Communication is facilitated through the I²C bus interface, ensuring seamless compatibility with popular microcontrollers such as Arduino.

![image](https://github.com/user-attachments/assets/acecf7d4-294f-4c13-ab1f-0c596922740d)

## Recording Data From MPU-6050
To leverage the capabilities offered by the MPU-6050 IMU, microcontroller platforms must be employed. One such option is the Arduino Uno board featuring the ATmega328P microcontroller. In order to capture angular velocities and accelerations from the sensor, a custom electronics design is necessary. Since the MPU-6050 supports communication via the I²C protocol, circuits will be developed accordingly.
For streamlined interaction with the MPU-6050 sensor utilizing Arduino, library-based methods are recommended. Two prominent libraries, namely ”Adafruit MPU6050” and ”MPU6050 tockn,” facilitate straightforward retrieval of calibrated sensor data. After integrating these libraries into your project, creating an instance of the MPU-6050 class becomes essential. With the object established, users may employ library functions to extract sensor data effortlessly. For instance, the ”Adafruit MPU6050” library provides functions like ‘getAcceleration()‘ and ‘getGyro()‘, while
the ”MPU605 tockn” library supplies functions like ‘getAccX()‘, ‘getAccY()‘, ‘getAccZ()‘, ‘getGyroX()‘, ‘getGyroY()‘, and ‘getGyroZ()‘. Additionally, these libraries offer functionalities such as sensor calibration, filtering, and sensor fusion, which further enhance the usability of the sensor data. By leveraging these libraries, developers can rapidly obtain acceleration, gyroscope, and temperature data without having to deal with intricate register-level details. This approach is beginner-friendly, expedites implementation, and suits a broad range of applications that do not necessitate fine-tuning at the register level.

![image](https://github.com/user-attachments/assets/2febd477-bb1f-4641-9ef1-52d6fcf4e04d)

## Calibration
To ensure accurate readings from MPU-6050 sensors, calibration is imperative to eliminate zeroerror or inherent inaccuracies in the sensor data. Given the unique nature of each sensor, calibration becomes essential as the default readings from the MPU6050 may not be entirely precise. By calibrating the sensor, users can attain more reliable representations of angular acceleration values and effectively mitigate zero-error discrepancies. The calibration procedure entails determining specific offset values for the sensor by placing it in a flat, level position and calculating three offsets that align the initial sensor readings closer to zero. This process can be facilitated through established libraries such as ‘MPU6050 tockn‘ or ‘Adafruit MPU6050‘, which offer user-friendly functions for retrieving calibrated sensor data. The obtained calibration values can then be integrated into the initialization code at the beginning of any sketch utilizing the MPU-6050 sensor. Calibration serves as a fundamental step in achieving precision in sensor readings and is crucial for applications that demand accurate measurements. By calibrating the MPU-6050 sensor, users can enhance the reliability and accuracy of their data outputs, ensuring optimal performance across various applications. 

### Tasks
1. Implement the circuit using Arduino-Uno and MPU-6050 to record the yaw, pitch, and roll
angles, also the quaternion value. Record a video file from the function of your system.

*** 
![image](https://github.com/user-attachments/assets/f43f6143-9f35-406d-975d-811127952a8e)

y and z can be found by rotating the sensor counterclockwise around the shown vectors. In such a way that the vector that produces positive yaw is z and the vector that produces positive pitch is the y axis.

The figure below is the graph produced in the Serial plotter, on which the three data of roll, pitch, and yaw are drawn.

![image](https://github.com/user-attachments/assets/feb89114-bcc7-4315-a57c-f3462a013534)


***

# Problem 2: Angle Filtering
Signal processing employs filters, which serve to eradicate undesirable elements or characteristics from signals. A distinguishing trait of filters lies in their ability to either completely or partially suppress specific aspects of the signal. Typically, filters target the removal of certain frequencies or frequency bands, although they aren’t confined solely to the frequency domain. Particularly in the realm of image processing, correlations can be eliminated selectively for specific frequency components without necessarily acting in the frequency domain. Filters permeate various domains, encompassing electronics, telecommunications, radio, television, audio recording, radar, control systems, music synthesis, image processing, computer graphics, and structural dynamics. Classification schemes for filters are multifarious and overlapping, rendering a concise taxonomy elusive. Filters may be categorized as linear or nonlinear, time-variant or time-invariant, causal or non-causal, analog or digital, discrete-time (sampled) or continuoustime, passive or active type of continuous-time filter. Commonly applied filters in signal processing consist of low-pass filters, high-pass filters, complementary filters, Kalman filters, finite impulse response (FIR) filters, and particle filters. Each filter type fulfills distinct purposes, and selecting the appropriate filter relies upon the specific requirements of the application.

### First Order Complementary Filter

![image](https://github.com/user-attachments/assets/8d359aad-1dc6-4068-8e37-ead21da420c4)

A complementary filter is a signal processing technique used with IMUs like the MPU-6050 to combine data from the accelerometer and gyroscope to obtain more accurate orientation estimates while minimizing drift. The complementary filter blends accelerometer data (sensitive to linear acceleration and gravity) and gyroscope data (measuring angular velocity) to produce a more stable and accurate estimation of orientation. The filter typically involves calculating pitch, roll, and yaw angles using both accelerometer and gyroscope data. By combining these measurements with appropriate weighting factors, the complementary filter provides a more reliable orientation estimate compared to using either sensor data alone. The filter helps reduce noise, improving accuracy, and minimizing drift in orientation estimation. By combining accelerometer and gyroscope data effectively, it provides a more stable output suitable for applications requiring precise orientation tracking. The implementation of the complementary filter involves processing raw sensor data, applying filtering algorithms, and updating orientation estimates in real time. The filter can be implemented in Arduino using code snippets that calculate pitch angles using gyroscope data and accelerometer readings. The formula for the output of this filter is as follows.

![image](https://github.com/user-attachments/assets/244e0d10-2790-4e92-8edd-ec2679520113)

### Kalman Filter
The Kalman filter is a powerful recursive algorithm that can be used to fuse data from multiple sensors, including the MPU-6050 sensor, to obtain more accurate orientation estimates. The Kalman filter accounts for measurement uncertainty and process noise, making it suitable for improving accuracy and reducing errors in orientation estimation when combined with data from the accelerometer and gyroscope of the MPU-6050 sensor. The filter involves predicting the state of the system based on the previous state and the current measurement, and then updating the state estimate based on the difference between the predicted and measured values. The filter also estimates the covariance of the state estimate, which can be used to quantify the uncertainty in the estimate. To implement a Kalman filter for the MPU-6050 sensor in Arduino, you can use pre-built libraries available on platforms like GitHub or the Arduino Library Manager. For example, the ”Arduino-KalmanFilter” library provides a simple implementation of the Kalman filter for the MPU-6050 sensor. The library includes a sample code that demonstrates how to use the filter to estimate the pitch and roll angles of the sensor. The code reads the raw accelerometer and gyroscope data from the sensor, applies the Kalman filter to the data, and then outputs the filtered pitch and roll angles. Keep in mind that implementing a Kalman filter requires more computational resources than a complementary filter or low-pass filter, so optimization techniques must be employed to prevent slowdowns. Additionally, tuning the filter parameters, such as the process noise covariance matrix and measurement noise covariance matrix, can be challenging and requires a good understanding of the system dynamics and noise characteristics.

### Tasks
1. Perform the first-order complementary filter algorithm on the code in the previous problem.
2. Perform the Kalman filter algorithm on the code in the previous problem. Using these links,
you can access proper Kalman libraries in Arduino Uno: [Link 1 ](https://www.arduino.cc/reference/en/libraries/kalman-filter-library/) and [Link 2](https://github.com/rfetick/Kalman).
3. Compare the results of both filters and discuss.
4. Record a video of the function of your system.

***
In this section, we filtered the pitch angle, which is shown below, as you can see, with the fast and unknown movement of the sensor, it shows the data correctly, also the speed of its changes is better.

![image](https://github.com/user-attachments/assets/445971f5-c0e1-40e3-98a7-79ebf74a00c2)

This section also uses the Kalman filter and we used two angles for this section. Pitch and roll angles:

![image](https://github.com/user-attachments/assets/85af93f5-f7e6-472a-b881-e8071c0089ca)

As you can see in the picture, with the sudden change of the sensor, the serial plotter shows the changes clearly and quickly. In general, the Kalman filter is a better and more convenient option.

***
# Problem 3: MPU-6050 as a Game Controller
## Overview
The goal of this problem is to create an interactive maze game controlled by physically moving and orienting an MPU-6050 accelerometer/gyroscope sensor. You will use the motion data from the sensor to update and control the position of a ball within a maze environment generated in Python. The provided Python code in this [Link](https://github.com/Sinakzm1379/Robotics_Course_TA) ( use the notebook in the MP1 folder ) utilizes the PyGame library to render graphics and handle gameplay for the maze. The code generates walls (rendered in cool blue) comprising the simple square maze layout along with a movable gray ball for the player avatar. Currently, this gray ball is controlled by the WASD keyboard keys to demonstrate basic movement mechanics. Pressing W makes the ball move up, A makes it move left, S down, and D right. The player uses these keys to navigate the ball through the openings in the walls and
try to reach the end of the maze.

![image](https://github.com/user-attachments/assets/3364a44d-1d0b-4668-b623-3ceb12f0c482)

Your task will be to replace this keyboard control scheme with input from the motion sensor. By tilting and rotating the MPU-6050, you will adjust the ball’s position on the screen. This creates a more physical and hands-on interaction for controlling the game compared to just pressing keys. The existing collision detection on the maze walls is still used to prevent moving through walls. Processing this analog motion data and mapping sensor orientation to screen position requires calculating directionality vectors and scaling factors. The existing collision detection code can still be leveraged to prevent illegal passage through maze walls. Just the control mechanism is replaced from key presses to motion tilting.
### Tasks
1. Connect the Arduino board to Python and read the data using Python.
2. Modify the Python game code to receive serial input from the Arduino instead of WASD keys. Map tilt motions to ball movement directions.
3. Demonstrate the working game by recording a video tilting the sensor to navigate the ball out of the maze (No longer than 30 seconds).

***
In this section, we must use Python to read the data of yaw roll pitch angles, to make sure that it is connected to Python.

![image](https://github.com/user-attachments/assets/17aa3e9b-756d-4a13-a0fa-bee69e5fd957)

In this section, we have to change the code of the maze game, which normally works with WASD keys, so that we can move in the maze without touching the WASD keys and only by changing the direction of the sensor in different directions.

![image](https://github.com/user-attachments/assets/307a15dd-174b-40f4-9e7e-f27d334ad0f4)

***
# Problem 4: Motion Capturing Using MediaPipe
MediaPipe is an open-source framework developed by Google for creating customizable machine-learning solutions for live and streaming media. It supports various platforms, including mobile (Android and iOS), web, desktop, edge devices, and Internet of Things (IoT). MediaPipe enables developers to rapidly prototype and deploy computer vision applications, offering ready-to-use solutions for tasks such as face detection, face mesh, iris detection, hands detection, and pose estimation. The framework is designed to be lightweight yet efficient, allowing it to run on battery-powered devices. MediaPipe uses a graph-based architecture consisting of nodes, packets, and graphs, providing flexibility and scalability for diverse applications. MediaPipe Pose is a machine-learning solution for high-fidelity body pose tracking, inferring 33 3D landmarks and background segmentation masks on the whole body from RGB video frames. It is a part of the MediaPipe framework developed by Google for creating customizable machine-learning solutions for live and streaming media. MediaPipe Pose uses a graph-based architecture consisting of nodes, packets, and graphs, providing flexibility and scalability for diverse applications. The solution is designed to be lightweight yet efficient, allowing it to run on battery-powered devices. MediaPipe Pose is suitable for applications requiring precise body pose tracking, such as fitness tracking, augmented reality, and more. The solution is open-source and can be customized to meet specific requirements. In this problem, you will be working with MediaPipe’s pose detection to capture the human leg’s motion and represent it using what you have learned from motion representation in the robotics course.

![image](https://github.com/user-attachments/assets/f8cf4a45-79c4-45c6-b18b-8d8764d5fa8c)

![image](https://github.com/user-attachments/assets/596fd6d8-a84d-41c8-be46-58fd41c0f84f)

### Tasks
Imagine that you want to capture and analyze the 2D rotational motion of a football player’s leg in a video, using MediaPipe Pose. Due to the inaccuracy of Mediapipe in-depth estimation, you have to record the data using two views. Therefore, it is suggested to record the motion from both lateral and frontal views of the leg simultaneously. Using these links, you can have a better knowledge of how this model should be implemented: [Link 1 (https://developers.google.com/mediapipe/solutions/vision/pose_landmarker), [Link 2](https://github.com/google-ai edge/mediapipe/blob/master/docs/solutions/pose.md),
1. Record two videos from the lateral and frontal view of the leg during a random rotational motion (No longer than 15 seconds). (Note: Take care to use two cameras with the same frame rates.)
2. For the leg (link between ankle and knee), compute the values θx and θy for each frame and form the rotation matrix.
3. Compute the vector ⃗e and the value of quaternion at each frame.
4. Plot the values θx, θy, and quaternion with respect to time.
5. The plots in the previous item represent that your observation is noisy. Save the data and perform an offline smoothening process. Re-plot the results and compare them with the previous item. (Note: You can use functions in MATLAB for this process.)

***
In this part of the project, we should be able to extract the angles of rotation around the y and x-axis between our ankle and knee by using Mediapipe and the video we take of our leg rotation.
In this section, we use two cameras, one from the front view and one from the side, to capture the rotation angles around the two axes.

![image](https://github.com/user-attachments/assets/8afe31d6-e233-4dc1-b0cf-c37d04b6c5b9)

In this section, we must obtain the angle between the knee and the ankle.

![image](https://github.com/user-attachments/assets/0faf9464-5931-4610-9e85-c8aaa36faed5)

Due to the video and sensor, some parts of the plot are noisy.

![image](https://github.com/user-attachments/assets/5bf211d4-a211-4516-a885-5a7e2e24ba47)

![image](https://github.com/user-attachments/assets/15b083f3-5768-4729-ad6f-dc649beb10d0)

It changes at any moment.

![image](https://github.com/user-attachments/assets/a5cf1939-fa7e-49f9-918f-a0e2ea2493bf)

![image](https://github.com/user-attachments/assets/62ea9e17-bd22-4b15-98ef-cbb72bad21e1)

![image](https://github.com/user-attachments/assets/3fde2385-f9c4-4aa7-957b-ddf29099e50f)

At this stage, we can use many methods included in MATLAB to display data without noise.
We used the Gaussian method to represent the data without noise.

![image](https://github.com/user-attachments/assets/78c26a09-00a5-44de-973e-15179c095bae)

![image](https://github.com/user-attachments/assets/11ae494c-c1c0-45d7-b251-e45b4f92d1bb)

![image](https://github.com/user-attachments/assets/bb5345d6-c880-4c5f-98cd-01987d4ac425)

***
