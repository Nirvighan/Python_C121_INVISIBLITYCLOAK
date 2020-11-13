import cv2
import time
import numpy as np

# PREPARATION FOR WRITING THE OUTPUT VIDEO

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc,20.0,(640,480))

# START READING THE VIDEO FROM WEBCAM

cap = cv2.VideoCapture(0)

# ALLOW THE WEBCAM TO STOP FOR 3 SECONDS AND THEN START

time.sleep(3)

count = 0
background = 0

# CAPTURE THE BACKGROUND IN RANGE OF 60

for i in range(60):
    ret,background = cap.read()

# FLIP THE BACKGROUND

background = np.flip(background,axis = 1)

# READ EVERY FRAME FROM THE WEBCAM UNTIL IT IS ON

while(cap.isOpened()):
    ret,img = cap.read()
    if not ret:
        break 

    count+=1
    img = np.flip(img,axis = 1)

    # CONVERT THE COLORS SPACE FROM GBR TO HSV
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    # GENERATE MASK TO DETECT THE COLOR

    # THE COLOR CAN BE ANY BUT YOU NEED TO GIVE THE RANGE ACCORDING TO THAT
    # THIS IS FOR LIGHTER COLOR (RED)
    lower_red = np.array([0,120,50])
    upper_red = np.array([10,255,255])

    mask1 = cv2.inRange(hsv,lower_red,upper_red)

    # THIS IS FOR Darker COLOR (RED),
    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])

    mask2 = cv2.inRange(hsv,lower_red,upper_red)

    # ADD BOTH THE MASKS TO GET THE MIDDLE SHADE OF COLOR
    mask1 = mask1+mask2

    # OPEN AND DILATE THE MASK IMAGE

    mask1 = cv2.morphologyEx(mask1,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))

    mask1 = cv2.morphologyEx(mask1,cv2.MORPH_DILATE,np.ones((3,3),np.uint8))

    # CREATE THE INVERTED MASK TO SEGMENT OUT THE RED COLOR FROM THE FRAME

    mask2 = cv2.bitwise_not(mask1)
    res1 = cv2.bitwise_and(img,img,mask = mask2)

    # CREATE IMAGE SHOWING STATIC BACKGROUNDFRAME PIXELS ONLY FOR THE MASK REGION

    res2 = cv2.bitwise_and(background,background,mask = mask1)

    # GENERATE THE FINAL OUTPUT

    final_res = cv2.addWeighted(res1,1,res2,1,0)
    out.write(final_res)
    cv2.imshow("MAGIC",final_res)
    cv2.waitKey(1)

cap.release()
out.release()

cv2.destroyAllWindows()

