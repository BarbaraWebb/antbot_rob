import  cv2 as cv
import cv2.cv as opencv

print(cv.__version__)

central = cv.imread('image_from_calibrate.png')
hsv = cv.cvtColor(central, cv.COLOR_BGR2HSV)
h = hsv[:,:,0]
s = hsv[:,:,1]
v = hsv[:,:,2]

cv.imshow('Hue', h)
cv.imshow('Sat', s)
cv.imshow('Val', v)

gray = cv.cvtColor(central, cv.COLOR_BGRA2GRAY)

blurred = cv.blur(s, (5,5))
gray = blurred
cv.imshow('Gray', gray)
circles = cv.HoughCircles(
    gray,
    opencv.CV_HOUGH_GRADIENT,
    1,
    20,
    param1=30,
    param2=250,
    minRadius=0,
    maxRadius=0
)

for i in circles[0,:]:
    cX = i[0]
    cY = i[1]
    cv.circle(central,(i[0],i[1]), 280, (0, 255, 0), 3)

cv.circle(central, (527, 406), 12, (0,0,255), 2)
#cv.circle(central, (cX, cY), 140, (0,0,255), 2)
#cv.circle(central, (cX, cY), 250, (0,0,255), 2)
cv.namedWindow('Central', cv.WINDOW_NORMAL)
cv.resizeWindow('Central', 600, 600)
cv.imshow('Central', central)

cv.waitKey(0)
cv.destroyAllWindows()
