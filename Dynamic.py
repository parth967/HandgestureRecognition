#!/usr/bin/env python
# coding: utf-8

# In[2]:


import cv2
import pyautogui, sys
import numpy as np
import win32api
import copy


def mouse_click(event,x,y,flags,param):
  if event==cv2.EVENT_LBUTTONDBLCLK:
    print ((x,y))
    color = tempFrame[y,x]
    blue = np.array_str(color[0])
    green = np.array_str(color[1])
    red = np.array_str(color[2])  
     
    color = np.uint8([[[blue, green, red]]])
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    global hue
    hue = hsv_color[0][0][0]

def secmouse_click(event,x,y,flags,param):
  if event==cv2.EVENT_LBUTTONDBLCLK:
    print ((x,y))
    color = tempFrame2[y,x]
    blue = np.array_str(color[0])
    green = np.array_str(color[1])
    red = np.array_str(color[2])  
     
    color = np.uint8([[[blue, green, red]]])
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    global hue2
    hue2 = hsv_color[0][0][0]
    

#Video Capturing
cam = cv2.VideoCapture(0)
count = 0

#First Color Cloth Detection for Cursor Movement
while True:
  r,tempFrame = cam.read()
  tempFrame = cv2.flip(tempFrame,1)
  cv2.imshow('Detection',tempFrame)
  if cv2.waitKey(1) & 0xFF == ord('d'):
    break

cv2.destroyWindow('Detection')
cv2.imshow("Color Picker",tempFrame)
cv2.setMouseCallback("Color Picker",mouse_click,tempFrame)
if cv2.waitKey(0) & 0xFF==ord('a'):
  cv2.destroyWindow("Color Picker")

#Second Color Cloth Detection For Double Click
while True:
  r,tempFrame2 = cam.read()
  tempFrame2 = cv2.flip(tempFrame2,1)
  cv2.imshow('Detection',tempFrame2)
  if cv2.waitKey(1) & 0xFF == ord('d'):
    break
cv2.destroyWindow('Detection')
cv2.imshow("Color Picker",tempFrame2)
cv2.setMouseCallback("Color Picker",secmouse_click,tempFrame2)
if cv2.waitKey(0) & 0xFF==ord('a'):
  cv2.destroyWindow("Color Picker")

#Video Display and other operations
while True:
  n, fra = cam.read();
  fra = cv2.resize(fra, (1366,768))
  fra = cv2.flip(fra,1)
  fra2 = fra

  #convert BGR to HSV
  hsv = cv2.cvtColor(fra, cv2.COLOR_BGR2HSV)
  hsv2 = cv2.cvtColor(fra2, cv2.COLOR_BGR2HSV)

  #Finding range for HSV
  lower_range = np.array([(hue-10), 100,  100], dtype=np.uint8)
  upper_range = np.array([(hue+10), 255, 255], dtype=np.uint8)
  seclower_range = np.array([(hue2-10), 100,  100], dtype=np.uint8)
  secupper_range = np.array([(hue2+10), 255, 255], dtype=np.uint8)


  #Applying Lower and Upper Range to HSV Frame
  mask = cv2.inRange(hsv, lower_range, upper_range)
  mask2 = cv2.inRange(hsv2, seclower_range, secupper_range)
  #cv2.imshow("HSV Frame",mask)
  
  #Applying Blur Effect to Remove Edges
  blb = cv2.bilateralFilter(mask, 10, 40, 10)
  blb2 = cv2.bilateralFilter(mask2, 10, 40, 10)
  #cv2.imshow("Bilateral Blur",blb)
  #Parameters - 1. Frame, 2. Range of neighbouring pixels, 3. Pixels having 40% of same intensity then they will be blurred
  # 4. 10 is the blur value

  #Applying Adaptive Threshold
  thres = cv2.adaptiveThreshold(blb, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21 , 1)
  thres2 = cv2.adaptiveThreshold(blb2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21 , 1)
  #cv2.imshow("Threshold",thres)
  #Parameters - 1.Frame, 2. Color to be filled, 3. Guassian Threshold Used, 4.Binary Threshold Type
  # 5. Block Size to be used for calculating image pixel value, 6. Default 1 Constant is applied  

  #Detecting Edges
  edged = cv2.Canny(thres, 100, 200)
  edged2 = cv2.Canny(thres2, 100, 200)
  #cv2.imshow("Edges",edged)
  #Parameters - 1. Frame, 2. Lower Range of Threshold, 3. Upper Range of Threshold


  #Contours Generation
  im21, contours, hierarchy1 = cv2.findContours(copy.deepcopy(edged), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  im22, contours2, hierarchy2 = cv2.findContours(copy.deepcopy(edged2), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  #Parameters - 1. Frame, 2. Retrieves all of the contours and reconstructs a full hierarchy of nested contours 
  # 3. Contour Approximation Method-By Default CHAIN_APPROX_SIMPLE

  #Finding maximum Area to avoid extra Contours
  maxArea=-1
  if len(contours)>0:
    for i in range(len(contours)):
      temp = contours[i]
      area = cv2.contourArea(temp)
      if area > maxArea:
        maxArea = area
        ci = i
    cnt = contours[ci]

    #Rectangle to Bound Object and draw it to display
    x,y,w,h = cv2.boundingRect(cnt)
    cv2.rectangle(fra,(x,y),(x+w,y+h),(0,255,0),2)

    #Cursor Movement
    win32api.SetCursorPos((x,y))

  if len(contours2)>0:
    
    cnt2 = contours2[0]

    #Rectangle to Bound Object and drawn to display
    x2,y2,w2,h2 = cv2.boundingRect(cnt2)
    cv2.rectangle(fra,(x2,y2),(x2+w2,y2+h2),(255,0,0),2)

    #Mouse Double Click
    pyautogui.click(clicks=2)
    
  cv2.imshow('Main',fra)

  #Key 'q' to quit
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

#Release the Camera resource and close all windows
cam.release()
cv2.destroyAllWindows()


# In[ ]:




