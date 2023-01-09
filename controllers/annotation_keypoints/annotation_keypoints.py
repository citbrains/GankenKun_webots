#!/usr/bin/env python3

import os
import sys
from controller import Robot, CameraRecognitionObject, Display, Supervisor
import random
import cv2
import numpy as np

deviceImagePath = os.getcwd()
#robot = Robot()
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
camera = supervisor.getDevice('camera_sensor')
camera.enable(timestep)
camera.recognitionEnable(timestep)
camera.enableRecognitionSegmentation()
number = 0

#print("hasRecognition(): " + str(camera.hasRecognition()))
#print("hasRecognitionSegmentation(): " + str(camera.hasRecognitionSegmentation()))

#cv2.startWindowThread()
#cv2.namedWindow("preview")

color_dict = { \
    #"ball_color"    :(0.0, 0.0, 1.0), \
    "colorHead"     :(1.0, 1.0, 1.0), \
#    "colorNeck"     :(0.5, 0.5, 0.5), \
    "colorShoulderL":(0.0, 0.499, 0.0), \
    "colorShoulderR":(0.0, 0.0, 0.499), \
    "colorElbowL"   :(0.0, 0.599, 0.0), \
    "colorElbowR"   :(0.599, 0.0, 0.0), \
    "colorWristL"   :(0.0, 0.0, 1.0), \
    "colorWristR"   :(0.7, 0.0, 0.0), \
    "colorHipL"     :(0.0, 0.8, 0.0), \
    "colorHipR"     :(0.8, 0.0, 0.0), \
    "colorKneeL"    :(0.0, 0.899, 0.0), \
    "colorKneeR"    :(0.899, 0.0, 0.0), \
    "colorAnkleL"   :(0.0, 1.0, 0.0), \
    "colorAnkleR"   :(1.0, 0.0, 0.0) \
}
#pos_dict = dict()
lines = [
    ["colorHead"     , "colorNeck"     ],
    ["colorNeck"     , "colorShoulderL"],
    ["colorShoulderL", "colorElbowL"   ],
    ["colorElbowL"   , "colorWristL"   ],
    ["colorNeck"     , "colorShoulderR"],
    ["colorShoulderR", "colorElbowR"   ],
    ["colorElbowR"   , "colorWristR"   ],
    ["colorNeck"     , "colorHipL"     ],
    ["colorHipL"     , "colorKneeL"    ],
    ["colorKneeL"    , "colorAnkleL"   ],
    ["colorNeck"     , "colorHipR"     ],
    ["colorHipR"     , "colorKneeR"    ],
    ["colorKneeR"    , "colorAnkleR"   ]
]

cnt = -1

memomemo_x = 0
memomemo_y = 0
memo_x = 0
memo_y = 0

while supervisor.step(timestep) != -1:
    # supervisor.getFromDef('BALL').getField('translation').setSFVec3f([random.uniform(0.0, 4.0), random.uniform(-1.0, 1.0), 0.1])
    # supervisor.getFromDef('ENEMY1').getField('rotation').setSFRotation([0, 0, 1, random.uniform(-1.0, 1.0)+3.14])
#    for i in range(10):
#        supervisor.step(timestep)
    camera.saveImage(deviceImagePath + '/images/image' + str(number) + '.jpg', 80)
    #camera.saveRecognitionSegmentationImage(deviceImagePath + '/images/segmentation_image' + str(number) + '.jpg', 80)
    number += 1
    seg_img = camera.getRecognitionSegmentationImage()
    img = np.frombuffer(seg_img, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    cnt +=1
    file_path = 'txt/image' + str(cnt) + '.txt'


    for key, val in color_dict.items():
        color = (val[0]*255, val[1]*255, val[2]*255, 255)
        area = cv2.inRange(img, color, color)
        x, y, w, h = cv2.boundingRect(area)
        #cv2.rectangle(img, (x, y), (x + width, y + height), color=color, thickness=2)
        #pos_dict[key] = (x + width / 2, y + height / 2)
        #print(key + ": " + str(x) + ", y: " + str(y) + ", with: " + str(width) + ", height: " + str(height))
        #print(key + ": " + str(x) + ", y: " + str(y))
        x = x + (w-1)/2
        y = y + (h-1)/2
        if x == 0 or y ==0: 
            x = memomemo_x 
            y = memomemo_y
            y += 20

        with open(file_path, 'a') as f:
            f.write("%s" % int(x))
            f.write(',')
            f.write("%s" % int(y))
#            if int(x)!=0 or int(y)!=0:
            f.write(',2\n')
#            else:
#                f.write(',0\n')
        memomemo_x = memo_x
        memomemo_y = memo_y
        memo_x = x
        memo_y = y
#    if cnt >= 600: 
#        print("anno_finish\n")
#        sys.exit()

    # for line in lines:
    #     x0, y0 = int(pos_dict[line[0]][0]), int(pos_dict[line[0]][1])
    #     x1, y1 = int(pos_dict[line[1]][0]), int(pos_dict[line[1]][1])
    #     print("x0: "+str(x0)+", y0: "+str(y0)+", x1: "+str(x1)+", y1: "+str(y1)+"\r\n")
    #     cv2.line(img, (x0, y0), (x1, y1), color=(255, 255, 255, 255), thickness=2)

    # cv2.imshow("preview", img)
    # cv2.waitKey(timestep)

