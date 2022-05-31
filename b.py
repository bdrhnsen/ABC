from cmath import pi
from re import X
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
import math
import serial
import cv2.aruco as aruco
import numpy as np
TURNDIST_MIN = 30 #Distance when to turn you can change this
turnMARKERID = 0 #Change this
accumulator = 0 #This variable will increase everytime we detect the marker with ID turnMARKERID less than 30 cm'
DIRECTION_BUFFER = [0,0,0,0,0,0,0]
def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()
    cv_file.release()
    return [camera_matrix, dist_matrix]

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_4X4_50",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())



# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
#	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
#	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
#	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
#	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
#	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
#	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
#	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
#	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
#	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
#	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
#	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
#	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
#	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
#	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
#	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
#	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
#	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
while(1):
    detected=0
    for i in range(10):
        try:
            ser = serial.Serial('/dev/ttyUSB'+str(i), 9600, timeout=1)
            detected=1
            break
        except:
            pass
    if detected==1:
        break
# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)

# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(1.0)

#Serial connection
#ser = serial.Serial('/dev/ttyACM0', 9600, 0.1) #120 bytes can be sent in 0.1 seconds.

# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 600 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=1000)

	# detect ArUco markers in the input frame
	(corners, ids, rejected) = cv2.aruco.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
		arucoDict, parameters=arucoParams)

	# verify *at least* one ArUco marker was detected
	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		found = 0
		for marker in ids: 
			if(marker == turnMARKERID) :
				found = 1
		if(found == 0):
			accumulator = 0
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners

			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			# draw the bounding box of the ArUCo detection
			cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

			# compute and draw the center (x, y)-coordinates of the
			# ArUco marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)


			# draw the ArUco marker ID on the frame
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			
			params = load_coefficients("calibration_params")
			cameraMatrix = params[0]
			distCoeffs = params[1]
			rvec, tvec, _ = aruco.estimatePoseSingleMarkers(markerCorner, 10, cameraMatrix, distCoeffs)
			x = tvec[0][0][0]
			y = tvec[0][0][1]
			z = tvec[0][0][2]
			print(tvec)
			print(x)
			print(y)
			print(z)
			dist = math.sqrt(x**2 + z**2)+20
			angle = -math.atan(x/z)
			angle = (angle*180/pi)+10
			print("ANGLE:" , angle)
			print("DIST: ", dist) 
			DIRECTION_BUFFER[0] = DIRECTION_BUFFER[1] 
			DIRECTION_BUFFER[1] = DIRECTION_BUFFER[2]
			DIRECTION_BUFFER[2] = DIRECTION_BUFFER[3]
			DIRECTION_BUFFER[3] = DIRECTION_BUFFER[4]
			DIRECTION_BUFFER[4] = DIRECTION_BUFFER[5]
			DIRECTION_BUFFER[5] = DIRECTION_BUFFER[6]
			DIRECTION_BUFFER[6] = DIRECTION_BUFFER[7]
			if(angle > 0) : 
				DIRECTION_BUFFER[7] = 1
			else :
				DIRECTION_BUFFER[7] = -1
			direction_sum = 0
			for entry in DIRECTION_BUFFER : 
				direction_sum += entry
				
			if(markerID == turnMARKERID and dist < TURNDIST_MIN ):
				accumulator = accumulator + 1
			elif(markerID == turnMARKERID and dist >= TURNDIST_MIN) :
				accumulator = 0
            
			if(accumulator >= 10) : 
				if(direction_sum > 0) :
					direct_str = "r"
				else : 
					direct_str = "l"
					# when Arduino gets TURN and LEFT or RIGHT messages, it moves accordingly. 

			elif(accumulator < 10) :
				direct_str="q"
				#Section to send dist and angle to Arduino
			dist_str = str(dist)
			angle_str = str(angle)
			send_str = dist_str + "," + angle_str + "*" direct_str + "\n"
			print(send_str)
			ser.flush()
			ser.write(send_str.encode('utf-8'))


	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
