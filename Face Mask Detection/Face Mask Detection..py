import cv2
import numpy as np
from keras import models 
import glob

mymodel = models.load_model(r"C:\Users\Mohsen\Desktop\mp2\mp2\model.h5")
face_cascade = cv2.CascadeClassifier(r"C:\Users\Mohsen\Desktop\mp2\mp2\haarcascade_frontalface_default.xml")

print(mymodel.summary())

def mask_detection(img):
  img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  img_GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  faces = face_cascade.detectMultiScale(img_GRAY, scaleFactor = 1.3, minNeighbors = 5, minSize = (100,100))
  for (x,y,w,h) in faces:
    roi_BGR = img[y:y+h, x:x+w]
    roi_GRAY = img_GRAY[y:y+h, x:x+w]
    roi_RGB = img_RGB[y:y+h, x:x+w]
    dim = (150, 150)
    img_n = roi_RGB.astype('float32') / 255
    resized = cv2.resize(img_n, dim)
    resized1 = np.reshape(resized,(1,150,150,3))
    pred = mymodel.predict(resized1)
    if pred < 0.5:
      label = 'Mask'
      color = (0,255,0)
    else:
      label = 'No Mask'
      color = (0,0,255)
    cv2.putText(img, label, (x+10, y+h+15), cv2.FONT_HERSHEY_SIMPLEX,
                1, color, 4)
    cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
  cv2.imshow("real image",img)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if frame is None:
        break

    mask_detection(frame)

    if cv2.waitKey(30) == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()