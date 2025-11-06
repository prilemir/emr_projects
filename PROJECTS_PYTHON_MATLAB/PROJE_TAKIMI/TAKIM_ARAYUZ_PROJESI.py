#!/usr/bin/env python3
import sys, subprocess
import cv2
import qdarktheme
import pyautogui
import rospy
import numpy as np
from rake_gui.tika_main import Ui_MainWindow  # ui->py 
from PIL import Image
from cv_bridge import CvBridge
from pathlib import Path

from sensor_msgs.msg import Image
from rake_gpio.msg import Detection
from std_msgs.msg import Float32
import os


import PySide6
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6.QtGui import QPixmap, QFont
from PySide6.QtCore import QTranslator
"""

This program represents the main page of the Graphical User Interface for Bumble Bee. 
Running this code on it's own is not advised, you will most likely not be able 
to see datas published from ros topics. Instead, run tika_intro and reach this page through that. 

"""

# TODO : There used to be a tool representing the light
# system on Bumble Bee, indicating whether the spraying process
# has started or not. Had to be removed due to necessary topics
# not being written yet. This tool can be re-created.

p = Path(__file__).parents[2]
archive_path = p / "src" / "gui_archive"
width, height = pyautogui.size()
ip = "192.168.1.4" # Jetson IP, change if necessary.

class window2(QMainWindow):
    def __init__(self,app):
        super(window2, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.app = app
        self.setWindowTitle("Ana Sayfa")
        #self.ui.progress_bar.setValue(100)
        
        self.ui.button.clicked.connect(self.topics)
        self.ui.pushButton.clicked.connect(self.emr_translationenglish)
        self.ui.jetson_button.clicked.connect(self.send_ping)
        self.ui.pushButton_2.clicked.connect(self.closing_images)
        self.ui.pushButton_3.clicked.connect(self.emr_translationturkish)
        self.translator = QTranslator()  
        self.count=0
        self.count1=0
        
        

        # STYLING
        self.ui.rake_logo.setPixmap(QPixmap(str(archive_path/"rake_logo.png")))
        self.ui.tika.setPixmap(QPixmap(str(archive_path/"tika2.png")))
        self.ui.cam_stream_label.setFont(QFont("VCR OSD Mono", 15))
        self.ui.cam2_stream_label.setFont(QFont("VCR OSD Mono", 15))
        self.ui.rviz_label.setFont(QFont("VCR OSD Mono", 15))
        self.ui.gazebo_label.setFont(QFont("VCR OSD Mono", 15))
        self.ui.gazebo_f_label.setFont(QFont("VCR OSD Mono", 15))
        self.ui.gazebo_b_label.setFont(QFont("VCR OSD Mono", 15))
        self.ui.rake_gui_title.setFont(QFont("Helvetica Neue Medium Extended", 23))
        self.ui.jetson_button.setFont(QFont("VCR OSD Mono"))

        # VARIABLES
        # These are the sizing of the displays on the 6-sub windows 
        # at the center of the page. 
        self.w = 505
        self.h = 373
       
        self.box1 = self.ui.cam_stream_label
        self.box2 = self.ui.cam2_stream_label
        self.box3 = self.ui.rviz_label
        self.box4 = self.ui.gazebo_label
        self.box5 = self.ui.gazebo_f_label
        self.box6 = self.ui.gazebo_b_label
        self.mode = self.ui.mode

        self.cam1_msg = None
        self.cam2_msg = None
        self.rviz_msg = None
        self.gazebo1_msg = None
        self.gazebo2_msg = None
        self.gazebo3_msg = None
        self.distance_msg = None
        self.battery_msg = None
        self.isdetection=None

        self.cam1_map = None
        self.gazebo1_map = None
        self.gazebo2_map = None
        self.gazebo3_map = None
        self.status = 2

        self.bridge = CvBridge()

        self.other_topics()

    def send_ping(self):
        cmd = ['ping', '-c', '2', ip]

        try:
            p = subprocess.run(cmd, timeout=1.5)
            self.ui.jetson_button.setText('ACTIVE')
            self.ui.jetson_button.setStyleSheet(
                "color: white; background-color: rgb(0,255,0)")
        except subprocess.TimeoutExpired:
            self.ui.jetson_button.setText('INACTIVE')
            self.ui.jetson_button.setStyleSheet(
                "color: white; background-color: rgb(255,0,0)")
            
    def emr_translationenglish(self):
        self.count1=self.count1+1
        
        LOCAL_DIR = os.path.dirname(os.path.realpath(__file__))
        self.translator = QTranslator()
        
        self.translator.load(LOCAL_DIR + "/tika_translation_file.qm")
        self.app.installTranslator(self.translator)
        self.update_ui()
    
    def emr_translationturkish(self):
        self.count1=self.count1+1
        
        LOCAL_DIR = os.path.dirname(os.path.realpath(__file__))
        self.translator = QTranslator()
        
        self.translator.load(LOCAL_DIR + "/tika_translationfiletr.qm")
        self.app.installTranslator(self.translator)
        self.update_ui()


    def closing_images(self):
     self.count=0
     if self.sub is not None:
        self.sub.unregister()
         
      
     self.box1.setText("GÖRÜNTÜ YOK!")
     self.box1.setStyleSheet("background-color: rgb(148, 148, 148)")
    def update_ui(self):

        self.ui.retranslateUi(self)


    def topics(self):
        self.topic1 = "Logitech_image" # Operator camera stream.
        self.topic2 = "plant_output_image" # Plant detection camera stream.
        self.topic3 = "" # Bumble Bee's state in RViz
        self.topic4 = "stage_camera/image_raw" # Bumble Bee's state in Gazebo
        self.topic5 = "front_camera/image_raw" # Front camera stream in Gazebo
        self.topic6 = "bottom_camera/image_raw" # Bottom camera stream in Gazebo
        self.topic12="is_there_detection"

        if self.ui.check_box1.isChecked():
            self.count=self.count+1
            
            
            if self.count<=1:

              
             self.sub= rospy.Subscriber(self.topic1, Image, self.cam1_msg_callback)
             
            if 1<self.count:
                self.sub.unregister()
                self.sub=rospy.Subscriber(self.topic1, Image, self.cam1_msg_callback)

        if self.ui.check_box2.isChecked():
            rospy.Subscriber(self.topic2, Image, self.cam2_msg_callback)
            rospy.Subscriber(self.topic12,Detection,self.emr_msgcallback)
            if self.cam2_msg is None:
                self.box2.setText("GÖRÜNTÜ YOK!")
                self.box2.setStyleSheet("background-color: rgb(148, 148, 148)")

        if self.ui.check_box3.isChecked():
            rospy.Subscriber(self.topic3, Detection, self.rviz_msg_callback)
            if self.rviz_msg is None:
                self.box3.setText("GÖRÜNTÜ YOK!")
                self.box3.setStyleSheet("background-color: rgb(148, 148, 148)")

        if self.ui.check_box4.isChecked():
            rospy.Subscriber(self.topic4, Image, self.gazebo1_msg_callback)
            rospy.Subscriber(self.topic5, Image, self.gazebo2_msg_callback)
            rospy.Subscriber(self.topic6, Image, self.gazebo3_msg_callback)
            if self.gazebo1_msg is None:
                self.box4.setText("GÖRÜNTÜ YOK!")
                self.box4.setStyleSheet("background-color: rgb(148, 148, 148)")
                
            if self.gazebo2_msg is None:
                self.box5.setText("GÖRÜNTÜ YOK!")
                self.box5.setStyleSheet("background-color: rgb(148, 148, 148)")
            if self.gazebo3_msg is None:
                self.box6.setText("GÖRÜNTÜ YOK!")
                self.box6.setStyleSheet("background-color: rgb(148, 148, 148)")

    # Distance travelled and the battery level.
    def other_topics(self):
        self.topic7 = "/total_distance" 
        self.topic8 = "/battery_level"
        rospy.Subscriber(self.topic7, Float32, self.distance_callback)
        rospy.Subscriber(self.topic8, Float32, self.battery_callback)

    # CALLBACK FUNCTIONS FOR EACH TOPIC
    def cam1_msg_callback(self, msg1):
        self.cam1_msg = msg1
        if self.cam1_msg is not None:
            cv_image1 = self.bridge.imgmsg_to_cv2(self.cam1_msg, "bgr8")
            self.cam1_map = cv_image1
            self.cam1_map = cv2.cvtColor(self.cam1_map, cv2.COLOR_BGR2RGB)
            height1, width1, channels1 = np.shape(self.cam1_map)
            totalBytes1 = self.cam1_map.nbytes
            bytesPerLine1 = int(totalBytes1 / height1)
            self.qimg1 = PySide6.QtGui.QImage(
                self.cam1_map.data, self.cam1_map.shape[1], self.cam1_map.shape[0], bytesPerLine1, PySide6.QtGui.QImage.Format_RGB888)
            self.pixmap1 = PySide6.QtGui.QPixmap.fromImage(self.qimg1)
            self.newpixmap1 = self.pixmap1.scaled(self.w, self.h)
            self.cam1_slot(self.newpixmap1)

    def cam2_msg_callback(self, msg2):
        self.cam2_msg = msg2
        if self.cam2_msg is not None:
            cv_image2 = self.bridge.imgmsg_to_cv2(self.cam2_msg, "bgr8")
            self.cam_map2 = cv_image2
            self.cam_map2 = cv2.cvtColor(self.cam_map2, cv2.COLOR_BGR2RGB)
            height2, width2, channels2 = np.shape(self.cam_map2)
            totalBytes2 = self.cam_map2.nbytes
            bytesPerLine2 = int(totalBytes2 / height2)
            self.qimg2 = PySide6.QtGui.QImage(
                self.cam_map2.data, self.cam_map2.shape[1], self.cam_map2.shape[0], bytesPerLine2, PySide6.QtGui.QImage.Format_RGB888)
            self.pixmap2 = PySide6.QtGui.QPixmap.fromImage(self.qimg2)
            self.newpixmap2 = self.pixmap2.scaled(self.w, self.h)
            self.cam2_slot(self.newpixmap2)

    def emr_msgcallback(self,data):
        self.isdetection=data.detectionData
        self.font= QFont("Arial", 10)
        self.ui.lineEdit.setFont(self.font)

        if self.isdetection is True:
            if (self.count1%2)==1:
             self.ui.lineEdit.setText("HERB İS DETECTED")
             self.ui.lineEdit.setStyleSheet("color: green; background-color: white")
            else:
                self.ui.lineEdit.setText("BİTKİ TESPİT EDİLDİ")
                self.ui.lineEdit.setStyleSheet("color: green; background-color: white")
            

        else:
            self.ui.lineEdit.setText("Bitki Tespit Edilmedi")
            self.ui.lineEdit.setStyleSheet("color: red; background-color: white")



    def rviz_msg_callback(self, msg3):
        self.rviz_msg_callback = msg3
        if self.rviz_msg_callback is not None:
            self.rviz_slot(self.rviz_msg_callback)

    def gazebo1_msg_callback(self, msg4):
        self.gazebo1_msg = msg4
        if self.gazebo1_msg is not None:
            cv_image1 = self.bridge.imgmsg_to_cv2(self.gazebo1_msg, "bgr8")
            self.gazebo1_map = cv_image1
            self.gazebo1_map = cv2.cvtColor(self.gazebo1_map, cv2.COLOR_BGR2RGB)
            height4, width4, channels4 = np.shape(self.gazebo1_map)
            totalBytes4 = self.gazebo1_map.nbytes
            bytesPerLine4 = int(totalBytes4 / height4)
            self.qimg4 = PySide6.QtGui.QImage(
                self.gazebo1_map.data, self.gazebo1_map.shape[1], self.gazebo1_map.shape[0], bytesPerLine4, PySide6.QtGui.QImage.Format_RGB888)
            self.pixmap4 = PySide6.QtGui.QPixmap.fromImage(self.qimg4)
            self.newpixmap4 = self.pixmap4.scaled(self.w, self.h)
            self.gazebo1_slot(self.newpixmap4)

    def gazebo2_msg_callback(self, msg5):
        self.gazebo2_msg = msg5
        if self.gazebo2_msg is not None:
            cv_image5 = self.bridge.imgmsg_to_cv2(self.gazebo2_msg, "bgr8")
            self.gazebo2_map = cv_image5
            self.gazebo2_map = cv2.cvtColor(
                self.gazebo2_map, cv2.COLOR_BGR2RGB)
            height5, width5, channels5 = np.shape(self.gazebo2_map)
            totalBytes5 = self.gazebo2_map.nbytes
            bytesPerLine5 = int(totalBytes5 / height5)
            self.qimg5 = PySide6.QtGui.QImage(
                self.gazebo2_map.data, self.gazebo2_map.shape[1], self.gazebo2_map.shape[0], bytesPerLine5, PySide6.QtGui.QImage.Format_RGB888)
            self.pixmap5 = PySide6.QtGui.QPixmap.fromImage(self.qimg5)
            self.newpixmap5 = self.pixmap5.scaled(self.w, self.h)
            self.gazebo2_slot(self.newpixmap5)

    def gazebo3_msg_callback(self, msg6):
        self.gazebo3_msg = msg6
        if self.gazebo3_msg is not None:
            cv_image6 = self.bridge.imgmsg_to_cv2(self.gazebo3_msg, "bgr8")
            self.gazebo3_map = cv_image6
            self.gazebo3_map = cv2.cvtColor(
                self.gazebo3_map, cv2.COLOR_BGR2RGB)
            height6, width6, channels6 = np.shape(self.gazebo3_map)
            totalBytes6 = self.gazebo3_map.nbytes
            bytesPerLine6 = int(totalBytes6 / height6)
            self.qimg6 = PySide6.QtGui.QImage(
                self.gazebo3_map.data, self.gazebo3_map.shape[1], self.gazebo3_map.shape[0], bytesPerLine6, PySide6.QtGui.QImage.Format_RGB888)
            self.pixmap6 = PySide6.QtGui.QPixmap.fromImage(self.qimg6)
            self.newpixmap6 = self.pixmap6.scaled(self.w, self.h)
            self.gazebo3_slot(self.newpixmap6)

    def distance_callback(self, msg7):
        self.distance_msg = msg7
        if self.distance_msg is not None:
            self.distance_slot(self.distance_msg)

    def battery_callback(self, msg8):
        self.battery_msg = msg8
        if self.battery_msg is not None:
            self.battery_slot(self.battery_msg)

    # SLOTS
    def cam1_slot(self, Image):
        self.box1.setPixmap(Image)
        self.box1.show()

    def cam2_slot(self, Image):
        self.box2.setPixmap(Image)
        self.box2.show()

    def rviz_slot(self, Detection):
        self.box3.setText(Detection)
        self.box3.show()

    def gazebo1_slot(self, Image):
        self.box4.setPixmap(Image)
        self.box4.show()

    def gazebo2_slot(self, Image):
        self.box5.setPixmap(Image)
        self.box5.show()

    def gazebo3_slot(self, Image):
        self.box6.setPixmap(Image)
        self.box6.show()

    def distance_slot(self, Num):
        self.ui.km_show.display(round(Num.data,2))
    
    def battery_slot(self,Battery):
        self.ui.progress_bar.setText("%"+str(round(Battery.data,1)))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    qdarktheme.setup_theme()
    
    window2 = window2(app)
    window2.resize(width, height)
    window2.show()
    sys.exit(app.exec_())
