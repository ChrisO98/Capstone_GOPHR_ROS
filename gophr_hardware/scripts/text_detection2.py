#!/usr/bin/env python3

# Import required packages
import rospy
from std_msgs.msg import String
from imutils.object_detection import non_max_suppression
import numpy as np
import argparse
import time
import cv2
import pytesseract

# Mention the installed location of Tesseract-OCR in your system
#pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
count = 0
# create functions to use in main function
def image_room(number: String): #number: String
    pytesseract.pytesseract.tesseract_cmd = r'/usr/bin/tesseract'
    # print the numer recieved from the user interface
    #print(number.data)

    # video capture
    cam = cv2.VideoCapture(0) # captureDevice = camera
    #cam = cv2.VideoCapture('/dev/video14', cv2.CAP_V4L) #cv2.CAP_V4L
    
    # set dimensions
    #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

    # If video cannot be opened then exit
    if not cam.isOpened():
        print("Error opening video")
        return

    #cv2.namedWindow("Logitech c270")
    img_counter = 0

    langs = (["en"])
    print("[INFO] OCR'ing with the following languages: {}".format(langs))
    
    count = 0
    wait_time = 0

    while cam.isOpened():
        # ret is success return and frame is the image captured frame by frame
        ret, frame = cam.read()
        # if no return then it failed
        if not ret:
            print("failed to grab frame")
            return
        # Video feed
        cv2.imshow("Logitech c270", frame)
        #time.sleep(10)
        #go = 32
        k = cv2.waitKey(1)
        current = rospy.Time.now()
        
        if count == 0:
            print(current.secs)
            wait_time = current.secs + 10
            print(wait_time)
            count = count + 1
        #print(count)

        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif wait_time <= current.secs: # go == 32
        #elif k%256 == 32: # SPACE pressed

            count = 0
            
            # load the input image and grab the image dimensions
            image = frame
            orig = image.copy()
            (H, W) = image.shape[:2]
            # set the new width and height and then determine the ratio in change
            # for both the width and height
            (newW, newH) = (320, 320)
            rW = W / float(newW)
            rH = H / float(newH)
            # resize the image and grab the new image dimensions
            image = cv2.resize(image, (newW, newH))
            (H, W) = image.shape[:2]
            
            # define the two output layer names for the EAST detector model that
            # we are interested -- the first is the output probabilities and the
            # second can be used to derive the bounding box coordinates of text
            layerNames = [
                "feature_fusion/Conv_7/Sigmoid",
                "feature_fusion/concat_3"]
	            
	    # load the pre-trained EAST text detector
            print("[INFO] loading EAST text detector...")
            net = cv2.dnn.readNet('frozen_east_text_detection.pb')#"~/catkin_ws/src/gophr_hardware/scripts/frozen_east_text_detection.pb"
            # construct a blob from the image and then perform a forward pass of
            # the model to obtain the two output layer sets
            blob = cv2.dnn.blobFromImage(image, 1.0, (W, H), (123.68, 116.78, 103.94), swapRB=True, crop=False)
            start = time.time()
            net.setInput(blob)
            (scores, geometry) = net.forward(layerNames)
            end = time.time()
            # show timing information on text prediction
            print("[INFO] text detection took {:.6f} seconds".format(end - start))
            
            # grab the number of rows and columns from the scores volume, then
            # initialize our set of bounding box rectangles and corresponding
            # confidence scores
            (numRows, numCols) = scores.shape[2:4]
            rects = []
            confidences = []
            # loop over the number of rows
            for y in range(0, numRows):
                # extract the scores (probabilities), followed by the geometrical
                # data used to derive potential bounding box coordinates that
                # surround text
                scoresData = scores[0, 0, y]
                xData0 = geometry[0, 0, y]
                xData1 = geometry[0, 1, y]
                xData2 = geometry[0, 2, y]
                xData3 = geometry[0, 3, y]
                anglesData = geometry[0, 4, y]
            
                # loop over the number of columns
                for x in range(0, numCols):
                    # if our score does not have sufficient probability, ignore it
                    if scoresData[x] < 0.5:
                        continue
                    # compute the offset factor as our resulting feature maps will
                    # be 4x smaller than the input image
                    (offsetX, offsetY) = (x * 4.0, y * 4.0)
                    # extract the rotation angle for the prediction and then
                    # compute the sin and cosine
                    angle = anglesData[x]
                    cos = np.cos(angle)
                    sin = np.sin(angle)
                    # use the geometry volume to derive the width and height of
                    # the bounding box
                    h = xData0[x] + xData2[x]
                    w = xData1[x] + xData3[x]
                    # compute both the starting and ending (x, y)-coordinates for
                    # the text prediction bounding box
                    endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
                    endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
                    startX = int(endX - w)
                    startY = int(endY - h)
                    # add the bounding box coordinates and probability score to
                    # our respective lists
                    rects.append((startX-5, startY-5, endX+5, endY+5))
                    #rects.append((startX-10, startY-10, endX+10, endY+10))
                    confidences.append(scoresData[x])
            # apply non-maxima suppression to suppress weak, overlapping bounding
            # boxes
            boxes = non_max_suppression(np.array(rects), probs=confidences)
            
            # A text file is created and flushed
            file = open("recognized.txt", "w+")
            file.write("")
            file.close()
            
            # loop over the bounding boxes
            for (startX, startY, endX, endY) in boxes:
                # scale the bounding box coordinates based on the respective
                # ratios
                startX = int(startX * rW)
                startY = int(startY * rH)
                endX = int(endX * rW)
                endY = int(endY * rH)
                # draw the bounding box on the image
                cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)
                # Cropping the text block for giving input to OCR
                cropped = orig[startY:endY, startX:endX]
                cv2.imwrite("croppedimage.png",cropped)
                
                # Open the file in append mode
                file = open("recognized.txt", "a")
                
                # Apply OCR on the cropped image
                text = pytesseract.image_to_string(cropped, config='--psm 10 --oem 3 -c tessedit_char_whitelist=abcdefghijklmnopqrstuvwxyz1234567890')
                
                # Appending the text into file
                file.write(text)
                file.write("\n")
	
                # Close the file
                file.close()
                
            with open('recognized.txt') as f:
                contents = f.read()
                print(contents)
                f.close()
                
            found = 0
            text_file = open('recognized.txt', "r")
            for aline in text_file:
                aline.split()
                string_value = aline[0:len(aline)-1]
                if string_value == number.data:
                    print(string_value)
                    text_string = String()
                    text_string.data = string_value
                    found = 1
                    pub_text.publish(text_string)
                    break
            text_file.close()
            if found == 0:
                print("No text detected")
                break
            
            break
    # When everything done, release the capture
    cam.release()
    # if a window was opened such as using 'imshow'
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    rospy.init_node("camera") # node name
    sub_number = rospy.Subscriber('cap_image', String, callback=image_room)
    pub_text = rospy.Publisher('image_text', String, queue_size=10)
    #image_room() #203
    rospy.spin()
