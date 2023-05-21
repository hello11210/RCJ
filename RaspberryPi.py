import cv2
import numpy as np
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1) #Serials setup to communicate with the megapi

vd = cv2.VideoCapture(0)

font = cv2.FONT_HERSHEY_SIMPLEX

def process(img, num): #Function for image processing (blurring, thresholding)
    #blurred = cv2.medianBlur(img, 15)
    if num == -1:
        blurred = cv2.GaussianBlur(img, (35, 35), 10)
        ret, thresh1 = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        edges = cv2.Canny(thresh1, 127, 200)
        return edges
    else:
        blurred = cv2.GaussianBlur(img, (3, 3), 10)
        ret, thresh1 = cv2.threshold(blurred, 30, 255, cv2.THRESH_BINARY_INV)
    return thresh1

def findCenter(contours, copy): #Find the center of contours using the moments method
    if len(contours) > 0:
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(copy, (cx, cy), 5, (225, 105, 65), -1)
            return cx, cy #Returns three values: the  x and y position and the indicator if there are contours
    return 0, 0

def checkGreen(imgHSV): #Check for green and return the contours
    lowGreen = np.array([65, 85, 51], dtype = np.uint8)
    #lowGreen = np.array([31,29,44],dtype = np.uint8) 
    highGreen = np.array([87, 205, 191], dtype = np.uint8)
    #highGreen = np.array([87,224,255],dtype = np.uint8) 
    thresh = cv2.inRange(imgHSV, lowGreen, highGreen)
    thresh = process(thresh, -1)
    cv2.imshow('G', thresh)
    conts, hier = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    return conts

def gap(img):
    sectionOne = process(img[240:480, :], 1)
    contours, hier = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow('GAP', sectionOne)
    #x1, y1 = findCenter(contours, copy) #Find the center of the line
    
def intersections(contours): #Check for green using inRange()
    #print("In intersections")
    i = 0
    numOfReals = 0
    greens = [0, 0]
    while i < len(contours):
        #area = cv2.contourArea(contours[i])
        #print("AREA: " + str(area))
        x, y, w, h = cv2.boundingRect(contours[i])
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        area = w * h
        #print("AREA: " + str(area))
        if area < 20.0:
            contours[i] = 0
        else:
            #print("Checking...")
            #print(str(i) + " " + str(area))
#             x, y, w, h = cv2.boundingRect(contours[i])
#             cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            #print(str(h) + " " + str(w))
            if(y < h//1):
                h = y
            mean = np.mean(frame[y - h//1 : y, x : x + w//1, :])
            #cv2.imshow("Temp", frame[y - h//1 : y, x : x + w//1, :])
#             rect = cv2.minAreaRect(contours[i])
#             print(rect[0][0])
#             y = int(rect[0][0])
#             x = int(rect[0][1])
#             new = frame[x - 80 : x + 80, y - 80: y + 80, :]
#             M = cv2.getRotationMatrix2D(rect[0], rect[2], 1)
#             new2 = cv2.warpAffine(new, M, (320, 320))
#             cv2.imshow("NEW2", new2)
#             box = np.int0(cv2.boxPoints(rect))
#             cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
            #print("Mean: " + str(mean))
            if mean < 120.0:
                if(x - 60 < 0):
                    lCheckFr = frame[y : y + h, 0 : x, :]
                else:
                    lCheckFr = frame[y : y + h, x - 60 : x, :]
                if np.mean(lCheckFr) < 80.0:
                    #print(np.mean(frame[y : y + h, x - 60 : x, :]))
                    cv2.imshow("Side Check", lCheckFr)
                    cv2.putText(frame, text = "Right", org = (x + 5, y + 70), fontFace = font,
                    fontScale = 1, color = (218, 165, 32), thickness = 1,
                    lineType = cv2.LINE_AA)
                    greens[0] = 1
                else:
                    cv2.putText(frame, text = "Left", org = (x + 5, y + 70), fontFace = font,
                    fontScale = 1, color = (218, 165, 32), thickness = 1,
                    lineType = cv2.LINE_AA)
                    greens[1] = 1
            else:
                cv2.putText(frame, text = "Fake!", org = (x + 5, y + 70), fontFace = font,
                fontScale = 1, color = (0, 0, 192), thickness = 1,
                lineType = cv2.LINE_AA)
        i+=1
    if greens[0] == 1 and greens[1] == 1:
        cv2.putText(frame, text = "U-Turn", org = (180, 310), fontFace = font,
        fontScale = 1, color = (130, 0, 75), thickness = 2,
        lineType = cv2.LINE_AA)
        #ser.write(('u' + '\n').encode('utf-8'))
        greens[1] = 1
        return 3 #3 for U-Turn
    elif greens[1] == 1:
        cv2.putText(frame, text = "Left", org = (180, 310), fontFace =- font,
        fontScale = 1, color = (130, 0, 75), thickness = 2,
        lineType = cv2.LINE_AA)
        #ser.write(('l' + '\n').encode('utf-8'))
        return 2 #2 for left
    elif greens[0] == 1:
        cv2.putText(frame, text = "Right", org = (180, 310), fontFace = font,
        fontScale = 1, color = (130, 0, 75), thickness = 2,
        lineType = cv2.LINE_AA)
        #ser.write(('r' + '\n').encode('utf-8'))
        return 1 #1 for right
    return 0 

ser.setDTR(False) #Serial setup stuff
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

def readFrames(gr): #Function to read in frames and convert them to HSV for green square processing
    error, frame = vd.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    #frame = frame[0:455, 90:530, :]
    if gr == 2:
        return frame
    frame = frame[0:340, :, :]
    #cv2.circle(frame, (320, 227), 5, (225, 105, 65), -1) #->>>>>>>>>>>>>>CENTER OF THE FRAME
    cpy = frame[100:, 100:540, :]
    #cpy = frame
    ##cv2.imshow("Copy", cpy)
    if gr == 0:
        img = cv2.cvtColor(cpy, cv2.COLOR_BGR2GRAY) #Convert the frame to grayscale for image processing
        imgHSV = cv2.cvtColor(cpy, cv2.COLOR_BGR2HSV) #Convert the frame to HSV for green square detection
    if gr == 1:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert the frame to grayscale for image processing
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #Convert the frame to HSV for green square detection
    return frame, imgHSV

def lineValid(fr):
    lineImgN = fr[200:, 200:440, :]
    #lineImgN = fr
    lineImg = process(cv2.cvtColor(lineImgN, cv2.COLOR_BGR2GRAY), 1)
    contours, hier = cv2.findContours(lineImg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    i = 0
    #cv2.imshow("LINE", lineImg)
    if(contours):
        cv2.drawContours(lineImgN, contours, -1, (0, 0, 255), 1)
        #linex, liney = findCenter(contours, lineImgN)
        #ser.write(('l' + '\n').encode('utf-8'))
        return True
    else:
        #print("NOT VALID")
        #ser.write(('w' + '\n').encode('utf-8'))
        return False
    cv2.imshow("LineImg", lineImgN)
#     if(np.mean(lineImgN) > 140.0):
#         print("VALID")      
#         ser.write(('w' + '\n').encode('utf-8'))
#     else:
#         lineImg = process(cv2.cvtColor(lineImgN, cv2.COLOR_BGR2GRAY), 1)
#         contours, hier = cv2.findContours(lineImg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
#         cv2.imshow("LINE", lineImg)
#         cv2.drawContours(lineImg, contours, -1, (0, 0, 255), 1)
#         if(contours):      
#             x1, y1 = findCenter(contours, lineImg)
#         if(x1 < 100 and x1 > 150):
#             print("Valid")
#             ser.write(('l' + '\n').encode('utf-8'))
#         else:
#             print("NOT VALID")
#             ser.write(('l' + '\n').encode('utf-8'))

def silverBall():
    sFrame = readFrames(2)
    sFrame = sFrame[0:400, :, :]
    #print(sFrame.shape)
    gray = cv2.cvtColor(sFrame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),0)
    gray = cv2.medianBlur(gray,5)
    #gray = cv2.Canny(gray, 50, 255)
    #gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
    #kernel = np.ones((3,3),np.uint8)
    #gray = cv2.erode(gray,kernel,iterations = 1)
    #gray = cv2.dilate(gray,kernel,iterations = 1)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 260, param1=30, param2=20, minRadius=150, maxRadius=200)
    if circles is not None:
        for x, y, r in circles[0]:
            #print("x: " + str(x) + " y: " + str(y) + " r: " + str(r))
            cv2.circle(sFrame, (int(x), int(y)), int(r), (255, 0, 0), 3)
            cv2.circle(sFrame, (int(x), int(y)), 5, (255, 0, 0), -1)
        #print(len(circles[0]))
        cv2.imshow("gray", gray)
        return circles[0], True, sFrame
    return circles, False, sFrame

def checkSilverTape(fra):
    tapeFrame = cv2.cvtColor(fra, cv2.COLOR_BGR2GRAY)
    ret, tapeThresh = cv2.threshold(tapeFrame, 250, 255, cv2.THRESH_BINARY)
    #cv2.imshow("Tape", tapeThresh)
    contours, hier = cv2.findContours(tapeThresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    r = len(contours)
    if(len(contours) > 0):
        for i in range(r):
            if cv2.contourArea(contours[i]) > 5000.0:
                print("Silver Tape Detected!   " + str(cv2.contourArea(contours[i])))
                return True
    #cv2.imshow("Tape Check", tapeFrame)
    return False

def ballPosition(circles, silFr):
    i = 0
    for x, y, r in circles:
        if(i == 1): #Purpose of this is to make sure we only grab the first values of x, y, r in circles[0]
            break
#         if(y > 275):
#             cv2.putText(silFr, text = "Back", org = (180, 310), fontFace =- font,
#         fontScale = 1, color = (130, 0, 75), thickness = 2,
#         lineType = cv2.LINE_AA)
#             ser.write(('vb' + '\n').encode('utf-8'))
#         elif(y < 50):
#             cv2.putText(silFr, text = "Forward", org = (180, 310), fontFace =- font,
#         fontScale = 1, color = (130, 0, 75), thickness = 2,
#         lineType = cv2.LINE_AA)
            #ser.write(('vf' + '\n').encode('utf-8'))
        if(x < 75):
            ser.write(('vt' + '\n').encode('utf-8'))
            print("Drastic right")
        elif(x < 280):
            cv2.putText(silFr, text =  "Right", org = (180, 310), fontFace =- font,
        fontScale = 1, color = (130, 0, 75), thickness = 2,
        lineType = cv2.LINE_AA)
            ser.write(('vr' + '\n').encode('utf-8'))
        elif(x > 605):
            print("Drastic left")
            ser.write(('vy' + '\n').encode('utf-8'))
        elif(x > 400):
            cv2.putText(silFr, text = "Left", org = (180, 310), fontFace =- font,
        fontScale = 1, color = (130, 0, 75), thickness = 2,
        lineType = cv2.LINE_AA)
            ser.write(('vl' + '\n').encode('utf-8'))
        else:
            cv2.putText(silFr, text = "Good", org = (180, 310), fontFace =- font,
        fontScale = 1, color = (130, 0, 75), thickness = 2,
        lineType = cv2.LINE_AA)
            ser.write(('vg' + '\n').encode('utf-8'))
        i+=1
    cv2.circle(silFr, (280, 200), 5, (255, 0, 255), -1)
    cv2.circle(silFr, (400, 200), 5, (255, 0, 255), -1)
    cv2.imshow("silver", silFr)

silverTape = True
firstSilver = False
cornerFound = False
squareCase = False
tCheck = False
while True:
    frame, imgHSV = readFrames(1)
#    print(lineValid(frame))
    if(silverTape == False):  
        silverTape = checkSilverTape(frame)
    if(silverTape):
        if(firstSilver == False):
            for i in range(10):
                ser.write(('si' + '\n').encode('utf-8'))
            print("Silver seen!")
            firstSilver = True
        if(cornerFound == False and ser.in_waiting > 0):
            line = ser.readline().decode('utf-8').rstrip()
            if(line == 'C'):
                #gconts = checkGreen(cv2.cvtColor(readFrames(2), cv2.COLOR_BGR2HSV))
                gconts = checkGreen(imgHSV)
                if(len(gconts) > 0):
                    print("Sending g")
                    ser.write(('g' + '\n').encode('utf-8'))
                    cornerFound = True
                else:
                    print("Sending n")
                    ser.write(('n' + '\n').encode('utf-8'))
        elif(cornerFound == True):
            silverC, silverCheck, silverFrame = silverBall()
            if(silverCheck):   
                ballPosition(silverC, silverFrame)
            else:
                ser.write(('vf' + '\n').encode('utf-8'))
    else: #if you don't see silver, do this
        #lineValid(frame)
#         #lineValid(frame)
#         if(ser.in_waiting > 0):
#             line = ser.readline().decode('utf-8').rstrip()
#             if(line == "I"):
#                 lineValid(frame)
        gconts = checkGreen(imgHSV)
        if(len(gconts) > 0): #If there is a green square
            #print("Green!")
            #x, y = findCenter(gconts, frame) #Find the center of the green square
            #print(y)
            #ser.write(('gs' + '\n').encode('utf-8'))
            #frame, imgHSV = readFrames(1)
            gconts = checkGreen(imgHSV)
            x, y = findCenter(gconts, frame)
    #         if(y > 350): #Drive until the green square is in the centeqr of the frame
    #              x, y = findCenter(gconts, frame)
    #              print("BACK")
    #              ser.write(('gb' + '\n').encode('utf-8')) #Tell the MegaPi to go backward
            #print("x: " + str(x) + " y: " + str(y))
            if(y > 300):
                ser.write(('gb5' + '\n').encode('utf-8'))
            elif(y < 150):
                x, y = findCenter(gconts, frame)
                ser.write(('gf' + '\n').encode('utf-8')) #Tell the MegaPi to go forward
            else:
                #print("GOOD")
                turn = intersections(gconts)
                ser.write(('gs' + str(turn) + '\n').encode('utf-8'))
        if(ser.in_waiting > 0 or tCheck):
            #print("IN")
            line = ser.readline().decode('utf-8').rstrip()
            if(line == "G"):
#                 print("tcheck is now false")
                tCheck = False
            if(line == "D" or tCheck):
                tCheck = True
                if(lineValid(frame)):
                    print("NOT A VALID SQUARE CASE")
                    ser.write(('W' + '\n').encode('utf-8'))
                    squareCase = False
                else:
                    print("IN SQUARECASE")
                    ser.write(('LA' + '\n').encode('utf-8'))
                    squareCase = True
                #print(turn)
    #         if(y < 100): #Drive until the green square is in the center of the frame
    #             x, y = findCenter(gconts, frame) 
    #             #print("Okay!")
    #             ser.write(('gf' + '\n').encode('utf-8')) #Tell the MegaPi to keep going forward
    #         else:
    #             turn = intersections(gconts)
    #             ser.write(('gs' + str(turn) + '\n').encode('utf-8')) #Tell the MegaPi to stop and turn a certain direction based on the green squares
    #         print("Green!")
        #else:
        #    ser.write(('x' + '\n').encode('utf-8'))
        #print(i)
        cv2.imshow("Frame", frame)
        
    if cv2.waitKey(1) == ord('q'):
        break
ser.close()
cv2.destroyAllWindows()
vd.release()



