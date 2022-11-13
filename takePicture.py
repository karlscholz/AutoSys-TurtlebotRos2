import cv2 as cv

cap = cv.VideoCapture(1)
ret, frame = cap.read()
cv.imwrite('image.png', frame)