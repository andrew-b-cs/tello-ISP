import time
from djitellopy import Tello
import cv2
import simple_pid
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

frameWidth = 640
frameHeight = 480

cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

initialize = True

me = Tello()
me.connect()
me.forward_backward_velocity = 0
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0



me.streamon()

while True:
    print(me.get_battery())
    frame_read = me.get_frame_read()
    height, width, _ = frame_read.frame.shape
    myFrame = frame_read.frame
    frame = cv2.resize(myFrame, (width, height))

    with mp_pose.Pose(min_detection_confidence=0.3, min_tracking_confidence=0.3) as pose:
    
        

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
    centerX = 0
    centerY = 0
    visibilityLeft = 0

    try:
        landmarks = results.pose_landmarks.landmark
        centerX = (landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x + landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x)/2
        centerY = (landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y + landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y)/2
        visibilityLeft = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].visibility
    except:
        pass
    
    print(visibilityLeft)

    if initialize:
        me.takeoff()
        initialize = False

    me.rotate_clockwise(90)

    # me.yaw_velocity = 99

    # me.send_rc_control(me.left_right_velocity, me.forward_backward_velocity, me.up_down_velocity, 99)
    
    # if me.get_battery() < 40:
    #     me.land()


    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        cv2.destroyAllWindows()
        break

        
