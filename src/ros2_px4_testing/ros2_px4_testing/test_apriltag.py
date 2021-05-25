import cv2
import numpy as np
from dt_apriltags import Detector

imagepath = 'src/ros2_px4_testing/ros2_px4_testing/img_apriltag.ppm'
img = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

#cv2.imshow('image',img)
#cv2.waitKey(0)

at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

tags = at_detector.detect(img, estimate_tag_pose=False, camera_params=None, tag_size=None)
#print(tags)

img_color = cv2.imread(imagepath)

start_point = (int(tags[0].corners[3][0]), int(tags[0].corners[3][1]))
end_point = (int(tags[0].corners[1][0]), int(tags[0].corners[1][1]))
center_x = int(tags[0].center[0])
center_y = int(tags[0].center[1])
img_color = cv2.rectangle(img_color, start_point, end_point, (0, 0, 255), 2)
img_color = cv2.circle(img_color, (center_x, center_y), 10, (0, 0, 255), -1)

cv2.imshow('image',img_color)
cv2.waitKey(0)
"""
pred_id_int=int(pred_id)
font = cv2.FONT_HERSHEY_SIMPLEX
org = (int(x1+5), int(y2-5))
fontScale = 0.9
img = cv2.putText(img, str(pred_id_int), org, font, fontScale, (0, 0, 255), 2, cv2.LINE_AA) 
"""