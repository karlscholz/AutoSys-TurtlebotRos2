import cv2 as cv
import mediapipe as mp
import time

mpDraw = mp.solutions.drawing_utils
mpPose = mp.solutions.pose
pose = mpPose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)


capture = cv.VideoCapture(0)
liveViewScale = 2
preTime = 0

while True:
    success, img = capture.read()
    imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    results = pose.process(imgRGB)
    #print(results.pose_landmarks)
    if results.pose_landmarks:
        mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
        for id, landmark in enumerate(results.pose_landmarks.landmark):
            h, w, c = img.shape
            cx, cy = int(landmark.x*w), int(landmark.y*h)
            if id == 0: #Nase
                cv.circle(img, (cx,cy), 5, (255,0,255), cv.FILLED)
                cv.putText(img, 'Nase', (cx, cy-5), cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255), 0)
            if id == 14: #Rechter Ellebogen
                cv.circle(img, (cx,cy), 5, (0,255,255), cv.FILLED)
                cv.putText(img, 'Rechter Ellebogen', (cx, cy-5), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 255), 0)

    curTime = time.time()
    fps = 1/(curTime-preTime)
    preTime = curTime


    cv.putText(img, str(int(fps)), (10,30), cv.FONT_HERSHEY_COMPLEX, 1.0, (255,255,0), 1)
    img = cv.resize(img, (img.shape[1] * liveViewScale,img.shape[0] * liveViewScale), interpolation=cv.INTER_AREA)
    #cv.imshow('VideoFeed', img)
    cv.imwrite("img_test.jpg",img)
    break
    if cv.waitKey(20) & 0xFF==ord('e'):
        capture.release()
        cv.destroyAllWindows()
        break