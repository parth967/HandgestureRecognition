#!/usr/bin/env python
# coding: utf-8

# In[1]:


import cv2
import numpy as np
import pyautogui, sys
import math
import copy
import win32ui as ui
import win32con as con
import win32gui as gui
import os

#threshold_val = 21
size = 8
arr = [-1]*size
k=-1

#Video Capturing
cam = cv2.VideoCapture(0)


def play_pause():
  pyautogui.press('space')

def previous_click():
  pyautogui.press('left')

def next_click():
  pyautogui.press('right')

def combine_click(cmd1,cmd2):
  pyautogui.hotkey(cmd1,cmd2)
  
def check():
  check_0 = 1
  other_0 = 1
  check_1 = 1
  other_1 = 1
  check_2 = 1
  other_2 = 2

  #Check for 0
  for i in range(0,size):
    print("Index["+str(i)+"]: "+str(arr[i]))
    for i in range(0,size):
      if arr[i] == 0:
        check_0 = check_0 + 1
      else:
        other_0 = other_0 +1

  #Check for 1
  for i in range(0,size):
    if arr[i] == 1:
      check_1 = check_1 + 1
    else:
      other_1 = other_1 +1

  #Check for 2
  for i in range(0,size):
    if arr[i] == 2:
      check_2 = check_2 + 1
    else:
      other_2= other_2 +1
      
  #Actual Check
  frontwin = str(gui.GetWindowText(gui.GetForegroundWindow()))
  
  #Play/Pause/Previous
  if check_0 > other_0:
    if "VLC media player" in frontwin:
      play_pause()
    if "Google Chrome" in frontwin:
      play_pause()
    if "iTunes" in frontwin:
      play_pause()
    if "PowerPoint Slide Show" in frontwin:
      next_click()
    print("Value Selected: 0")

  #Next 
  if check_1 > other_1:
    if "PowerPoint Slide Show" in frontwin:
      previous_click()
    print("Value Selected: 1")

  #Shutdown or Desktop
  if check_2 > other_2:
    #os.system('shutdown -s')
    #os.system('explorer D:\Hindi')
    combine_click('win','d')
    print("Value Selected: 2")
  

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

#Cloth Detection for perform basic operations
while True:
  n,tempFrame = cam.read()
  tempFrame = cv2.flip(tempFrame,1)
  #tempFrame = cv2.resize(tempFrame, (1366,768))
  cv2.imshow('Detection',tempFrame)
  
  if cv2.waitKey(1) & 0xFF == ord('d'):
    break
cv2.destroyWindow('Detection')
cv2.imshow('Picker',tempFrame)
cv2.setMouseCallback('Picker',mouse_click,tempFrame)
if cv2.waitKey(0) & 0xFF == ord('a'):
  cv2.destroyWindow('Picker')

#Video Display and other operations
while True:
    r, frame = cam.read()

    #crop_cut = frame[100:400, 150:450]
    crop_cut=frame

    #convert BGR to HSV
    crop_hsv = cv2.cvtColor(crop_cut,cv2.COLOR_BGR2HSV)
    #cv2.imshow("HSV Frame",crop_hsv)

    #Calculating Lower & Upper Range
    lower_range = np.array([(hue - 10), 100, 100], dtype=np.uint8)
    upper_range = np.array([(hue + 10), 255, 255], dtype=np.uint8)

    #Applying Lower and Upper Range to HSV Frame
    crop = cv2.inRange(crop_hsv, lower_range, upper_range)
    #cv2.imshow("HSV Frame",crop)

    #Applying Blur Effect to Remove Edges
    blb = cv2.bilateralFilter(crop, 10, 40, 10)
    cv2.imshow("Bilateral Blur",blb)
    #Parameters - 1. Frame, 2. Range of neighbouring pixels, 3. Pixels having 40% of same intensity then they will be blurred
    # 4. 10 is the blur value

    #Applying Adaptive Threshold
    thres = cv2.adaptiveThreshold(blb, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21 , 1)
    cv2.imshow("Threshold",thres)
    #Parameters - 1.Frame, 2. Color to be filled, 3. Guassian Threshold Used, 4.Binary Threshold Type
    # 5. Block Size to be used for calculating image pixel value, 6. Default 1 Constant is applied 

    #Detecting Edges
    edged = cv2.Canny(thres, 100, 200)
    cv2.imshow("Edges",edged)
    #Parameters - 1. Frame, 2. Lower Range of Threshold, 3. Upper Range of Threshold

    #Contours Generation
    im2, contours, hierarchy = cv2.findContours(copy.deepcopy(edged), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Parameters - 1. Frame, 2. Retrieves all of the contours and reconstructs a full hierarchy of nested contours 
    # 3. Contour Approximation Method-By Default CHAIN_APPROX_SIMPLE

    #Finding maximum Area to avoid extra Contours
    cv2.drawContours(crop_cut, contours, -1, (0, 0, 0), 1)
    maxArea=-1
    drawing = np.zeros(crop_cut.shape, np.uint8)
    if len(contours) > 0:
      
      for i in range(len(contours)):
        temp = contours[i]
        area = cv2.contourArea(temp)
        if area > maxArea:
          maxArea = area
          ci = i

      #Applying Obtained Contours as parameters to draw Convexity Hull and Calculate Defects
      cnt = contours[ci]
      hull2 = cv2.convexHull(cnt)
      hull = cv2.convexHull(cnt, returnPoints=False)
      
      #Draing calculated contours on frame
      cv2.drawContours(drawing, [cnt], 0 , (0,255,0),2)
      cv2.drawContours(drawing, [hull2], 0 , (0,0,255),2)

      #Calculating Defects
      defects = cv2.convexityDefects(cnt, hull)

      count_defects=0
      if defects is not None:
        for i in range(defects.shape[0]):
          s, e, f, d = defects[i, 0]
          start = tuple(cnt[s][0])
          end = tuple(cnt[e][0])
          far = tuple(cnt[f][0])

          #Math Operation to find Lengths
          a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
          b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
          c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

          #Finding angle between to Fingers
          angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) *57
          #57 is the Orientation of Hand

          #Ignore angles > 90 and highlight rest with red dots
          if angle <= 90:
            count_defects += 1
            
            #Drawing Red Circle to detected contours on Convexity Hull
            cv2.circle(drawing, far, 5, (0, 0, 255), -1)
            
        #Calculate maximum defects in an array    
        k=k+1
        arr[k] = count_defects
        if (k==(size-1)):
          check()
          k=0
          
        if count_defects==1:
            #Display 1, if only one defect is found
            cv2.putText(crop_cut, str(count_defects), (160, 110), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,255,255), 2)
        if count_defects==2:
            #Display 2, if only two defect is found
            cv2.putText(crop_cut, str(count_defects), (160, 110), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2)
        else:
            #Display total defect that are found
            cv2.putText(crop_cut, str(count_defects), (160, 110), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2)

    cv2.imshow("Main", crop_cut)

    #Key 'q' to quit
    if cv2.waitKey(1) & 0xFF==ord('q'):
      break

#Release the Camera resource and close all windows
cam.release()
cv2.destroyAllWindows()


# In[ ]:




