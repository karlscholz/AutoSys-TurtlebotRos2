import cv2 as cv
import time
import poseEstimationModule as pem


capture = cv.VideoCapture(0)
liveViewScale = 2
preTime = 0

detector = pem.poseDetector()
while True:
    success, img = capture.read()

    img = detector.findPose(img, draw=True)
    landmarkList = detector.findPosition(img, draw=False)
    if len(landmarkList) != 0:
        id=14#Rechter Ellebogen
        print(landmarkList[id])
        cv.circle(img, (landmarkList[id][1],landmarkList[id][2]), 7, (0,255,255), cv.FILLED)

    curTime = time.time()
    fps = 1/(curTime-preTime)
    preTime = curTime


    cv.putText(img, str(int(fps)), (10,30), cv.FONT_HERSHEY_COMPLEX, 1.0, (255,255,0), 1)
    img = cv.resize(img, (img.shape[1] * liveViewScale,img.shape[0] * liveViewScale), interpolation=cv.INTER_AREA)
    cv.imshow('VideoFeed', img)

    if cv.waitKey(20) & 0xFF==ord('e'):
        capture.release()
        cv.destroyAllWindows()
        break