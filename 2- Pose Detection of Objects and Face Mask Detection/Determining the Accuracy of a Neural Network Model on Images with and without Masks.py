import cv2
import numpy as np
from keras import models
import glob

mymodel = models.load_model(r"C:\Users\Mohsen\Desktop\mp2\mp2\model.h5")
face_cascade = cv2.CascadeClassifier(r"C:\Users\Mohsen\Desktop\mp2\mp2\haarcascade_frontalface_default.xml")

mask_images = [cv2.imread(file) for file in glob.glob(r"C:\Users\Mohsen\Desktop\mp2\mp2\archive\Mask\*.*")]
y_mask = np.zeros((np.shape(mask_images)[0],1))

no_mask_images = [cv2.imread(file) for file in glob.glob(r"C:\Users\Mohsen\Desktop\mp2\mp2\archive\NoMask\*.*")]
y_no_mask = np.ones((np.shape(no_mask_images)[0],1))

x_mask = mask_images.copy()
x_no_mask = no_mask_images.copy()

dim = (150, 150)
resized_mask = []
for i in range(np.shape(mask_images)[0]):
  if x_mask[i] is None:
    y_mask = np.delete(y_mask, np.shape(y_mask)[0]-1, axis = 0)
  else:
    x_mask_RGB = cv2.cvtColor(x_mask[i], cv2.COLOR_BGR2RGB)
    x_mask_GRAY = cv2.cvtColor(x_mask[i], cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(x_mask_GRAY, scaleFactor = 1.3, minNeighbors = 5, minSize = (50,50))
    if np.shape(faces)[0] == 0:
      y_mask = np.delete(y_mask,  np.shape(y_mask)[0]-1, axis = 0)
    else:
      if np.shape(faces)[0] > 1:
        y_mask = np.append(y_mask, np.zeros((np.shape(faces)[0]-1,1)), axis = 0)
      for (x,y,w,h) in faces:
        roi_RGB = x_mask_RGB[y:y+h, x:x+w]
        dim = (150, 150)
        resized = cv2.resize(roi_RGB, dim)
        img_n = resized.astype('float32') / 255
        resized_mask.append(resized)
resized_mask = np.array(resized_mask)

_, mask_accuracy = mymodel.evaluate(resized_mask, y_mask, verbose=0)
print('Test_accuracy for mask:', mask_accuracy * 100)

dim = (150, 150)
resized_no_mask = []
for i in range(np.shape(no_mask_images)[0]):
  if x_no_mask[i] is None:
    y_no_mask = np.delete(y_no_mask, np.shape(y_no_mask)[0]-1, axis = 0)
  else:
    x_no_mask_RGB = cv2.cvtColor(x_no_mask[i], cv2.COLOR_BGR2RGB)
    x_no_mask_GRAY = cv2.cvtColor(x_no_mask[i], cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(x_no_mask_GRAY, scaleFactor = 1.3, minNeighbors = 5, minSize = (70,70))
    if np.shape(faces)[0] == 0:
      y_no_mask = np.delete(y_no_mask,  np.shape(y_no_mask)[0]-1, axis = 0)
    else:
      if np.shape(faces)[0] > 1:
        y_no_mask = np.append(y_no_mask, np.ones((np.shape(faces)[0]-1,1)), axis = 0)
      for (x,y,w,h) in faces:
        roi_no_RGB = x_no_mask_RGB[y:y+h, x:x+w]
        dim = (150, 150)
        resized = cv2.resize(roi_no_RGB, dim)
        img_n = resized.astype('float32') / 255
        resized_no_mask.append(img_n)
resized_no_mask = np.array(resized_no_mask)

_, no_mask_accuracy = mymodel.evaluate(resized_no_mask, y_no_mask, verbose=0)
print('Test_accuracy for no mask:', no_mask_accuracy * 100)