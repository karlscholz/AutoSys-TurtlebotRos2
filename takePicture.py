import cv2 as cv

cap = cv.VideoCapture(0)
ret, frame = cap.read() # read a frame !
cv.imwrite('image.png', frame)
cap.release()