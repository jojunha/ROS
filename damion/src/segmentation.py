#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import rospy
import math
import glob
import os
import numpy as np
import cv2
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import array_to_img, img_to_array, load_img
import tensorflow as tf
from tensorflow.python.client import device_lib
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession
from cv_bridge import CvBridge

from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import Image
from damion.msg import object_info 

# gpu
config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)
# cpu
# os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

##상수 선언 ##
frame = np.empty(shape=[0])
pred = np.empty(shape=[0])
num = 0
max_ = 0
cur_lon = 0
cur_lat = 0
last_lon = 127
last_lat = 37
c_sub = False
m_sub = False
##상수 선언 ##  

def gps_callback(data):
    global cur_lon, cur_lat
    cur_lon = data.lon/(10000000.0)
    cur_lat = data.lat/(10000000.0)
    print("longitudinal :" , cur_lon)
    print("lateral :" , cur_lat)


def image_callback(data):
    global frame, c_sub
    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, 3)
    c_sub = True
    cv2.imshow("img",frame)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()


def check_gpu():    
    device_lib.list_local_devices()


def detect_crack():
    global frame
    global img
    global num, max_
    global pred
    global last_lon, last_lat, cur_lat, cur_lon
    rate = pred.sum()/(pred.shape[2]*pred.shape[1]*255)
    rate = rate*100
    if rate <= 100:
        if max_ <= rate:
            max_ = rate
    print("percent :",rate)
    if rate < 100:
        if rate >= 1: ## threshold 1%
            if (((cur_lon - last_lon)*1000000)**2 + ((cur_lat - last_lat)*1000000)**2)**0.5 > 48:
                print("detect crack")
                print("rate :", rate)
                name = "image"+ str(num)
                cv2.imwrite('/home/kong/catkin_ws/src/damion/Image/'+ name + ".png", frame)
                num = num+1
                last_lon = cur_lon
                last_lat = cur_lat
                gps_msgs = object_info()
                gps_msgs.latitude = last_lat
                gps_msgs.longitude = last_lon
                gps_msgs.class_name = "crack"
                gps_msgs.file_name = '/home/kong/catkin_ws/src/damion/Image/'+ name + ".png"
                gps_pub.publish(gps_msgs)
        else:
            print("normal road")
            print("rate :", rate)

## Model ##
def dice_coef(y_true, y_pred):
        y_true_f = K.flatten(y_true)
        y_pred_f = K.flatten(y_pred)
        intersection = K.sum(y_true_f * y_pred_f)
        return (2. * intersection + K.epsilon()) / (K.sum(y_true_f) + K.sum(y_pred_f) + K.epsilon())

def dice_coef_loss(y_true, y_pred):
        return 1-dice_coef(y_true, y_pred)

def sens(y_true, y_pred): # sensitivity, recall
    print(y_pred)
    print(y_true)
    y_target_yn = K.round(K.clip(y_true, 0, 1)) # 실제값을 0(Negative) 또는 1(Positive)로 설정한다
    y_pred_yn = K.round(K.clip(y_pred, 0, 1)) # 예측값을 0(Negative) 또는 1(Positive)로 설정한다

    # True Positive는 실제 값과 예측 값이 모두 1(Positive)인 경우이다
    count_true_positive = K.sum(y_target_yn * y_pred_yn) 

    # (True Positive + False Negative) = 실제 값이 1(Positive) 전체
    count_true_positive_false_negative = K.sum(y_target_yn)

    # Recall =  (True Positive) / (True Positive + False Negative)
    # K.epsilon()는 'divide by zero error' 예방차원에서 작은 수를 더한다
    recall = count_true_positive / (count_true_positive_false_negative + K.epsilon())

    # return a single tensor value
    return recall
## Model ##

## Video Show ##
def video_show(model):
    global c_sub
    global frame
    global pred
    if c_sub:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, (512, 512))
        img = np.float32(img)
        img = img / 255.0
        img = np.expand_dims(img, axis=0)
        pred = model.predict(img)
        pred[pred > 0.5] = 255
        pred[pred <= 0.5] = 0
        detect_crack()
        #cv2.imshow("img",frame)
        cv2.imshow("pred",pred[0])
        cv2.waitKey(1)
    c_sub = False

#       cv2.destroyAllWindows()


## Video Show ##



if __name__ == '__main__':
    rospy.init_node('segmentation')
    gps_pub = rospy.Publisher('/seg_info', object_info, queue_size = 1)
    camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    gps_sub = rospy.Subscriber("/ublox_msgs/navpvt", NavPVT, gps_callback)
    check_gpu()
    model = load_model('/home/kong/catkin_ws/src/damion/model/final_18_0.150126.hdf5',
                                custom_objects={'sens':sens,'dice_coef_loss': dice_coef_loss})
    # video_file = '/home/kong/catkin_ws/src/damion/Video/20220318_151355.mp4'
    # video_show(video_file, model)

    while not rospy.is_shutdown():
        #try:
            video_show(model)
        #except rospy.ROSInterruptException:
         #   pass