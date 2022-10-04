import time
from djitellopy import Tello
import cv2
from simple_pid import PID
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

frameWidth = 640
frameHeight = 480

cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

initialize = False

me = Tello()
me.connect()
me.for_back_velocity = 0
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0



me.streamoff()
me.streamon()

while True:
    print(me.get_battery())
    frame_read = me.get_frame_read()

    myFrame = frame_read.frame
    frame = cv2.resize(myFrame, (320, 240))

    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    
        

            # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        
        # Make detection
        results = pose.process(image)

        # Recolor back to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        # Render detections
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2) 
                                    )      

        cv2.imshow("MyResult", image)

    try:
        landmarks = results.pose_landmarks.landmark
        
    except:
        pass
 
    if initialize:
        me.takeoff()
        time.sleep(6)
        me.rotate_counter_clockwise(90)
        time.sleep(3)
        me.rotate_clockwise(90)
        me.land()
        initialize = False
    
    print(landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value])
    print(landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        cv2.destroyAllWindows()
        break
exit()
        
