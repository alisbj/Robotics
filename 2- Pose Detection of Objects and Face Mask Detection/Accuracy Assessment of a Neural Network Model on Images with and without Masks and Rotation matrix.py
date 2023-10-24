import cv2
import numpy as np
from math import *

img1 = cv2.imread(r"C:\Users\Mohsen\Desktop\mp2\mp2\Pictures\pic1.png")
img2 = cv2.imread(r"C:\Users\Mohsen\Desktop\mp2\mp2\Pictures\pic2.png")
img3 = cv2.imread(r"C:\Users\Mohsen\Desktop\mp2\mp2\Pictures\pic3.png")
img_blue = img1.copy()
img_red = img2.copy()
img_green = img3.copy()

cv2.imshow("Blue Cube",img_blue)
cv2.imshow("Red Cube",img_red)
cv2.imshow("Green Cube",img_green)
cv2.waitKey(0)
cv2.destroyAllWindows()

img_blueHSV = cv2.cvtColor(img_blue, cv2.COLOR_BGR2HSV)
img_redHSV = cv2.cvtColor(img_red, cv2.COLOR_BGR2HSV)
img_greenHSV = cv2.cvtColor(img_green, cv2.COLOR_BGR2HSV)
cv2.imshow("Blue HSV",img_blueHSV)
cv2.imshow("Red HSV",img_redHSV)
cv2.imshow("Green HSV",img_greenHSV)
cv2.waitKey(0)
cv2.destroyAllWindows()

blue = np.uint8([[[255,0,0 ]]])
red = np.uint8([[[0,0,255 ]]])
green = np.uint8([[[0,255,0 ]]])

hsv_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)
hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)

print('hsv blue:',hsv_blue)
print('hsv red:',hsv_red)
print('hsv green:',hsv_green)

img_blueThr = cv2.inRange(img_blueHSV, (110, 100, 100), (130, 255, 255))
img_redThr = cv2.inRange(img_redHSV, (0, 100, 100), (10, 255, 255))
img_greenThr = cv2.inRange(img_greenHSV, (50, 100, 100), (70, 255, 255))

cv2.imshow("Binary Blue Cube",img_blueThr)
cv2.imshow("Binary Red Cube",img_redThr)
cv2.imshow("Binary Green Cube",img_greenThr)
cv2.waitKey(0)
cv2.destroyAllWindows()

def getOrientation(pts, img):

  sz = len(pts)
  data_pts = np.empty((sz, 2), dtype=np.float64)
  for i in range(data_pts.shape[0]):
    data_pts[i,0] = pts[i,0,0]
    data_pts[i,1] = pts[i,0,1]
 
  mean = np.empty((0))
  mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
 
  cntr = (int(mean[0,0]), int(mean[0,1]))
  
  cv2.circle(img, cntr, 3, (255, 0, 255), 1)
 
  angle = atan2(eigenvectors[0,1], eigenvectors[0,0])
 
  angle_deg = -int(np.rad2deg(angle)) + 90
  if angle_deg > 0:
    angle_deg = angle_deg - 180 

  label = "  Rotation Angle: " + str(angle_deg) + " degrees"
  textbox = cv2.rectangle(img, (cntr[0]-40, cntr[1]-50), (cntr[0] + 230, cntr[1]-25), (255,255,255), -1)
  cv2.putText(img, label, (cntr[0]-50, cntr[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
  
  return angle_deg

def orientation(image,imageThreshold):
  img = image.copy()
  imgThr = imageThreshold.copy()
  contours, _ = cv2.findContours(imgThr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  for i, c in enumerate(contours):
      area = cv2.contourArea(c)
      cv2.drawContours(img, contours, i, (255, 255, 255), 1)
      phi = getOrientation(c, img)
  cv2.imshow("Angles",img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  return phi

phi_blue = orientation(img_blue,img_blueThr)
phi_red = orientation(img_red,img_redThr)
phi_green = orientation(img_green,img_greenThr)

def position(image,imageThreshold):
  img = image.copy()
  imgThr = imageThreshold.copy()
  M = cv2.moments(imgThr)
  cX = int(M["m10"] / M["m00"])
  cY = int(M["m01"] / M["m00"])
  pos = (cX,cY)
  label = "position:" + str(pos)
  cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
  textbox = cv2.rectangle(img, (cX - 25, cY - 40), (cX + 135 , cY - 15), (255,255,255), -1)
  cv2.putText(img, label, (cX - 20, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
  cv2.imshow("Position",img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()

position(img_blue,img_blueThr)
position(img_red,img_redThr)
position(img_green,img_greenThr)

Q_blue = np.array([[cos(radians(phi_blue)),-sin(radians(phi_blue)),0],
                    [sin(radians(phi_blue)),cos(radians(phi_blue)),0],[0,0,1]])

Q_green = np.array([[cos(radians(phi_green)),-sin(radians(phi_green)),0],
                    [sin(radians(phi_green)),cos(radians(phi_green)),0],[0,0,1]])

Q_red = np.array([[cos(radians(phi_red)),-sin(radians(phi_red)),0],
                    [sin(radians(phi_red)),cos(radians(phi_red)),0],[0,0,1]])

print('Q_blue = \n',Q_blue)
print('\nQ_red = \n',Q_red)
print('\nQ_green = \n',Q_green)

