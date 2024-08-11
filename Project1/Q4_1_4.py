import matplotlib.pyplot as plt
import numpy as np
import mediapipe as mp
import cv2
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

def calculate_joint_angle(b, c):
    b_np = np.array(b) 
    c_np = np.array(c) 

    radians = np.arctan2(b_np[1] - c_np[1], b_np[0] - c_np[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle - 90

video_file1 = "C:/Users/h0me/Desktop/mechat ca/IMG_8779.mp4" 
cap_x = cv2.VideoCapture(video_file1)
video_file2 = "C:/Users/h0me/Desktop/mechat ca/IMG_8780.mp4"
cap_y = cv2.VideoCapture(video_file2)

t = 0


with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap_x.isOpened():
        ret_x, frame_x = cap_x.read()

        
        image_x = cv2.cvtColor(frame_x, cv2.COLOR_BGR2RGB)
        image_x.flags.writeable = False

        
        results_x = pose.process(image_x)

        
        image_x.flags.writeable = True
        image_x = cv2.cvtColor(image_x, cv2.COLOR_RGB2BGR)
      
        try:
            landmarks_x = results_x.pose_landmarks.landmark

            knee_joint_x = [landmarks_x[mp_pose.PoseLandmark.RIGHT_KNEE.value].x,
                             landmarks_x[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
            ankle_joint_x = [landmarks_x[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x,
                              landmarks_x[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]

            angle_x = round(calculate_joint_angle(knee_joint_x, ankle_joint_x), 2)

        except:
            pass

        ret_y, frame_y = cap_y.read()

        image_y = cv2.cvtColor(frame_y, cv2.COLOR_BGR2RGB)
        image_y.flags.writeable = False

        results_y = pose.process(image_y)

        image_y.flags.writeable = True
        image_y = cv2.cvtColor(image_y, cv2.COLOR_RGB2BGR)
        try:
            landmarks_y = results_y.pose_landmarks.landmark

            # Get coordinates
            knee_joint_y = [landmarks_y[mp_pose.PoseLandmark.RIGHT_KNEE.value].x,
                            landmarks_y[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
            ankle_joint_y = [landmarks_y[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x,
                             landmarks_y[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]

            # Calculate angle_y
            angle_y = round(calculate_joint_angle(knee_joint_y, ankle_joint_y), 2)

        except:
            pass

        print("theta_x = ", angle_x, "theta_y = ", angle_y)
        # Calculate Rotation Matrix
        Q = [[math.cos(angle_y), math.sin(angle_x) * math.sin(angle_y), math.cos(angle_x) * math.sin(angle_y)],
             [0, math.cos(angle_x), -math.sin(angle_x)],
             [-math.sin(angle_y), math.sin(angle_x) * math.cos(angle_y), math.cos(angle_x) * math.cos(angle_y)]]
        #print("Q = ", Q)

        # Calculate e & phi
        tr = Q[0][0] + Q[1][1] + Q[2][2]
        V1 = 1 / 2 * Q[2][1] - Q[1][2]
        V2 = 1 / 2 * Q[0][2] - Q[2][0]
        V3 = 1 / 2 * Q[1][0] - Q[0][1]
        vect = [[V1], [V2], [V3]]
        phi = math.acos((tr - 1) / 2)
        E1 = 1 / 2 * V1 / math.sin(phi)
        E2 = 1 / 2 * V2 / math.sin(phi)
        E3 = 1 / 2 * V3 / math.sin(phi)
        e = [[E1], [E2], [E3]]
        #print("phi = ", phi, "e = ", e)

        # Calculate quaternion
        r0 = math.cos(phi / 2)
        R1 = math.sin(phi / 2) * E1
        R2 = math.sin(phi / 2) * E2
        R3 = math.sin(phi / 2) * E3
        r = [[R1], [R2], [R3]]
        print("r0 = ", r0, "r = ", r)

        # Plot theta_x
        try:
            t += 1
            amp_1 = angle_x
            plt.figure(1)
            plt.subplot(2, 1, 1)
            plt.scatter(t, amp_1)
            plt.title('Theta_x Value')
            plt.xlabel('Time')
            plt.ylabel('Theta_x')
            plt.pause(1)
        except:
            pass

        f = open("x.txt", "a")
        f.write(str(amp_1) + "\n")
        f.close()

        # Plot theta_y
        try:

            amp_2 = angle_y
            plt.subplot(2, 1, 2)
            plt.scatter(t, amp_2)
            plt.title('Theta_y Value')
            plt.xlabel('Time')
            plt.ylabel('Theta_y')
            plt.pause(1)
        except:
            pass

        f = open("y.txt", "a")
        f.write(str(amp_2) + "\n")
        f.close()

        # Plot quaternion
        try:
            amp_3 = r0
            plt.figure(2)
            plt.scatter(t, amp_3)
            plt.title('r_0 Value')
            plt.xlabel('Time')
            plt.ylabel('r_o')
            plt.pause(1)
        except:
            pass

        f = open("r0.txt", "a")
        f.write(str(amp_3) + "\n")
        f.close()

        # Render detections
        mp_drawing.draw_landmarks(image_x, results_x.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                  mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                  mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                  )
        mp_drawing.draw_landmarks(image_y, results_y.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                  mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                  mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                  )
        cv2.imshow('Mediapipe Feed', image_x)
        # cv2.imshow('Mediapipe Feed', image_y)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap_x.release()
    cv2.destroyAllWindows()
    plt.show()
