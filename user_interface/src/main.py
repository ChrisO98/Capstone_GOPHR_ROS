#!/usr/bin/env python3

from functools import partial
import sys
import rospy
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QMessageBox, QLabel, QDialogButtonBox
from PyQt5.QtGui import QFont
from PyQt5.QtCore import *
import images
import RPi.GPIO as gpio
from mfrc522 import SimpleMFRC522
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int16, String


gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

global room_num_sel
global pat_info
global nurse_info
global reason_des
global popup_new

class ThreadRFID(QThread):
    signal = pyqtSignal(str)
    successSignalRFID = pyqtSignal()
    success_signal_popup = pyqtSignal()
    
    def __del__(self):
        self.wait()
    
    def run(self):
        # Initialize RFID Reader
        reader = SimpleMFRC522()
        while True: # keep as a while true so the thread does not terminate
            global popup_new
            if popup_new == 1:
                id, text = reader.read()
                if(id == 358883595246):
                    print("scan card success")
                    self.success_signal_popup.emit()

            elif popup_new == 0:
                id, text = reader.read()
                if(id == 358883595246):
                    print("scan card success")
                    self.successSignalRFID.emit()
 
                
class GoalReached(QThread):
    signal = pyqtSignal(str)
    success_signal = pyqtSignal()
    
    def __del__(self):
        self.wait()
    
    def run(self):
        # setup rospy subscriber
        rospy.Subscriber('goal_reached', String, callback=self.goal_reached) # subscribed to uart
        self.pub_room = rospy.Publisher('cap_image', String, queue_size=10) # set up publisher for text_detection.py
    
    def goal_reached(self,msg: String):
        global room_num_sel
        if msg.data == '1':
            #self.success_signal.emit()
            number = String()
            number.data = room_num_sel
            self.pub_room.publish(number) # sends room number to text_detection.py
            
class CameraFeedback(QThread):
    signal = pyqtSignal(str)
    success_signal_cam = pyqtSignal()
    
    def __del__(self):
        self.wait()
    
    def run(self):
        # setup rospy subscriber
        rospy.Subscriber('image_text', String, callback=self.camera_feed) # subscribe to text_detection.py
    
    def camera_feed(self,msg: String):
        global room_num_sel
        if msg.data == room_num_sel:
            self.success_signal_cam.emit()
            
class PathComplete(QThread):
    signal = pyqtSignal(str)
    success_signal_path = pyqtSignal()
    
    def __del__(self):
        self.wait()
    
    def run(self):
        # setup rospy subscriber
        rospy.Subscriber('path_complete', String, callback=self.complete) # subscribe to text_detection.py
    
    def complete(self,msg: String):
        #global room_num_sel
        if msg.data == 'done':
            self.success_signal_path.emit()
            
class HomeBase(QThread):
    signal = pyqtSignal(str)
    success_signal_base = pyqtSignal()
    
    def __del__(self):
        self.wait()
    
    def run(self):
        # setup rospy subscriber
        rospy.Subscriber('home_base', String, callback=self.base) # subscribe to text_detection.py
    
    def base(self,msg: String):
        #global room_num_sel
        if msg.data == 'base':
            self.success_signal_base.emit()
            

class Authorize(QMainWindow):
    def __init__(self):
        super(Authorize, self).__init__()
        loadUi("/home/tjcc/catkin_ws/src/user_interface/src/ScanBadge.ui", self)
        global popup_new
        popup_new = 0
        #self.pushButton.clicked.connect(self.go_page2)
        self.my_thread = ThreadRFID(self)
        self.my_thread.start()
        self.my_thread.successSignalRFID.connect(self.go_page2)
        self.my_thread.success_signal_popup.connect(self.activate_servo)       


    # function will now change the window only if the currentIndex is that of an RFID screen
    def go_page2(self): 
        global nurse_info
        global pat_info
        global reason_des
        if(widget.currentIndex() == 0):
            widget.setCurrentIndex(1)
            # file contains nurse_data that is obtained from the badge being scanned using a RFID scanner
            file = "/home/tjcc/catkin_ws/src/user_interface/src/data/nurse_data.txt"
            self.first_nurse = open(file).read()
            nurse_info = self.first_nurse
            page2.textEdit_2.setText(self.first_nurse)
            
        elif(widget.currentIndex() == 3):
            if pat_info == None:
                pat_info = ""
            self.data_access = "\nNurse: %s\n\n\nPatient:\n%s\n\nReason: %s" % (nurse_info,pat_info,reason_des)
            page5.textEdit.setText(self.data_access)
            widget.setCurrentIndex(4)
    
    def activate_servo(self):
        global popup_new
        # Check if in Instruct class/page 2
        if widget.currentIndex() == 1 and popup_new == 1:
            servo_data = String()
            servo_data.data = 'Unlock'
            pub_servo.publish(servo_data)
            popup_new = 0
        
        elif widget.currentIndex() == 4 and popup_new == 1:
            servo_data = String()
            servo_data.data = 'Unlock'
            pub_servo.publish(servo_data)
            popup_new = 0


class MessageBox(QMessageBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        grid_layout = self.layout()

        qt_msgboxex_icon_label = self.findChild(QLabel, "qt_msgboxex_icon_label")
        qt_msgboxex_icon_label.deleteLater()

        qt_msgbox_label = self.findChild(QLabel, "qt_msgbox_label")
        qt_msgbox_label.setAlignment(Qt.AlignCenter)
        qt_msgbox_label.setStyleSheet("QLabel{min-width: 300px;min-height: 100px;font: 28pt 'Ubuntu';}")
        grid_layout.removeWidget(qt_msgbox_label)

        qt_msgbox_buttonbox = self.findChild(QDialogButtonBox, "qt_msgbox_buttonbox")
        qt_msgbox_buttonbox.setStyleSheet("QWidget{font: 20pt 'Ubuntu';dialogbuttonbox-buttons-have-icons: 1;}")
        grid_layout.removeWidget(qt_msgbox_buttonbox)

        grid_layout.addWidget(qt_msgbox_label, 0, 0, alignment=Qt.AlignCenter)
        grid_layout.addWidget(qt_msgbox_buttonbox, 1, 0, alignment=Qt.AlignCenter)

class Instruct(QMainWindow):
    def __init__(self):
        super(Instruct, self).__init__()
        loadUi("/home/tjcc/catkin_ws/src/user_interface/src/MakeDelivery.ui", self)
        global popup_new
        popup_new = 0

        room_selected = ""

        self.Lockbox = "Unlock" # for do_popup function

        def room_number(_str, patient_info = None, waypoint_file = None):
            global room_num_sel # used for text_detection2.py
            global pat_info
            if (patient_info == None) or (waypoint_file == None): # used to be and
                # SHOULD NEVER END UP HERE
                ##clear textEdit
                self.textEdit.setText("")
                self.lineEdit.setText(_str)
                self.room_selected = _str
                room_num_sel = _str # for text_detection2.py
                pat_info = patient_info
                self.number = Int16()
                self.number.data = 0
                
            else:
                self.lineEdit.setText(_str)
                text = open(patient_info).read()
                # pat_info used for pages 2 and 5
                pat_info = text
                self.textEdit.setText(text)

                # only need to publish a number based on the room selceted such as
                #  room 001 will be 1,
                #  room 002 will be 2, etc.
                
                # replace pose with float 64 or some integer message built into ROS
                self.number = Int16()
                self.number.data = waypoint_file
                self.room_selected = _str # variables with a self. that are not part of the PyQt
                # get room number to publish through ROS to text_detection2.py
                room_num_sel = _str
                # can be used in any function within the same class
        

        def reason(_str):
            global reason_des
            reason_des = _str
            self.lineEdit_2.setText(_str)
        
        self.toolButton_21.clicked.connect(lambda: reason("Documents"))
        self.toolButton_22.clicked.connect(lambda: reason("Supplies"))
        self.toolButton_23.clicked.connect(lambda: reason("Medication"))
        self.toolButton_24.clicked.connect(lambda: reason("Food"))

        self.toolButton.clicked.connect(lambda: room_number("001", "/home/tjcc/catkin_ws/src/user_interface/src/data/patient_data.txt", 1))
        self.toolButton_2.clicked.connect(lambda: room_number("002"))
        self.toolButton_3.clicked.connect(lambda: room_number("003"))
        self.toolButton_4.clicked.connect(lambda: room_number("004"))
        self.toolButton_5.clicked.connect(lambda: room_number("005"))
        self.toolButton_6.clicked.connect(lambda: room_number("006"))
        self.toolButton_7.clicked.connect(lambda: room_number("007"))
        self.toolButton_8.clicked.connect(lambda: room_number("008"))
        self.toolButton_9.clicked.connect(lambda: room_number("009"))
        self.toolButton_10.clicked.connect(lambda: room_number("010"))
        self.toolButton_11.clicked.connect(lambda: room_number("011"))
        self.toolButton_12.clicked.connect(lambda: room_number("012"))
        self.toolButton_13.clicked.connect(lambda: room_number("013"))
        self.toolButton_14.clicked.connect(lambda: room_number("014"))
        self.toolButton_15.clicked.connect(lambda: room_number("015"))
        self.toolButton_16.clicked.connect(lambda: room_number("016"))
        self.toolButton_17.clicked.connect(lambda: room_number("017"))
        self.toolButton_18.clicked.connect(lambda: room_number("018"))
        self.toolButton_19.clicked.connect(lambda: room_number("019"))
        self.toolButton_20.clicked.connect(lambda: room_number("020"))
    
        # add button for servo lockbox
        self.toolButton_25.clicked.connect(self.go_page1)
        self.toolButton_26.clicked.connect(self.go_page3)
        self.toolButton_27.clicked.connect(self.do_popup)

    # https://stackoverflow.com/questions/67476677/how-to-center-text-and-buttons-in-qmessagebox-widget
    def do_popup(self):
        #open box
        if self.Lockbox == "Unlock":
            # This function will show popup window to open box
            # then popup window will have RFIDThread signal that connects to a function activate_servo from Authorize class
            # the function will publish an 'unlock' to the servo
            global popup_new
            popup_new = 1

            pop = MessageBox()

            flags = QtCore.Qt.WindowFlags(QtCore.Qt.FramelessWindowHint | int(QtCore.Qt.WindowStaysOnTopHint))
            pop.setWindowFlags(flags)

            pop.setText("Scan Badge")
            pop.setStandardButtons(QMessageBox.Ok)

            pop.exec_()

    def go_page1(self):
        self.lineEdit_2.setText("")
        self.lineEdit.setText("")
        self.textEdit_2.setText("")
        self.textEdit.setText("")
        widget.setCurrentIndex(widget.currentIndex()-1)
    
    def go_page3(self):
        self.lineEdit_2.setText("")
        self.lineEdit.setText("")
        self.textEdit_2.setText("")
        self.textEdit.setText("")
        if self.number.data != 0:
            pub.publish(self.number) # publish room selected
        page3.label_2.setText(self.room_selected) # room number selected
        widget.setCurrentIndex(widget.currentIndex()+1)


class Traveling(QMainWindow):
    def __init__(self):
        super(Traveling, self).__init__()
        loadUi("/home/tjcc/catkin_ws/src/user_interface/src/TravelingToDestination.ui", self)

        self.pushButton.clicked.connect(self.go_page4)
        self.goal_thread = GoalReached(self)
        self.goal_thread.start()
        #self.goal_thread.success_signal.connect(self.go_page4)
        self.camera_thread = CameraFeedback(self)
        self.camera_thread.start()
        self.camera_thread.success_signal_cam.connect(self.go_page4)
        
        # move on passed this page when path is complete
        self.path_complete = PathComplete(self)
        self.path_complete.start()
        self.path_complete.success_signal_path.connect(self.go_page4)
    
    def go_page4(self):
        if(widget.currentIndex() == 2):
            widget.setCurrentIndex(widget.currentIndex()+1)


class SecondAuthorize(QMainWindow):
    def __init__(self):
        super(SecondAuthorize, self).__init__()
        loadUi("/home/tjcc/catkin_ws/src/user_interface/src/ScanBadge.ui", self)
        self.label.setText("Scan Badge\nTo Retrieve Item")
    
    def go_page5(self):
        file = "/home/tjcc/catkin_ws/src/user_interface/src/data/nurse_data.txt"
        self.first_nurse = open(file).read() # needs to be dynamic
        self.patient = open("/home/tjcc/catkin_ws/src/user_interface/src/data/patient_data.txt").read() # needs to be dynamic
        self.data_access = "\nNurse:\n%s\n\n\nPatient:\n%s" % (self.first_nurse,self.patient)
        page5.textEdit.setText(self.data_access)
        widget.setCurrentIndex(widget.currentIndex()+1)



class Retrieve(QMainWindow):
    def __init__(self):
        super(Retrieve, self).__init__()
        loadUi("/home/tjcc/catkin_ws/src/user_interface/src/RetrieveItem.ui", self)
        
        self.Lockbox = "Unlock" # for do_popup function

        self.toolButton.clicked.connect(self.do_popup) #self.get_item
        self.toolButton_2.clicked.connect(self.go_page6)
    
    def do_popup(self):
        #open box
        if self.Lockbox == "Unlock":
            # This function will show popup window to open box
            # then popup window will have RFIDThread signal that connects to a function activate_servo from Authorize class
            # the function will publish an 'unlock' to the servo
            global popup_new
            popup_new = 1

            pop = MessageBox()

            flags = QtCore.Qt.WindowFlags(QtCore.Qt.FramelessWindowHint | int(QtCore.Qt.WindowStaysOnTopHint))
            pop.setWindowFlags(flags)

            pop.setText("Scan Badge")
            pop.setStandardButtons(QMessageBox.Ok)

            pop.exec_()
    
    def get_item(self):
        #open box
        if self.item_text == "Unlock":
            servo_data = String()
            servo_data.data = self.item_text
            print(servo_data.data)
            self.item_text = "Lock"
            self.toolButton.setText("Lock Box")
            
            pub_servo.publish(servo_data)
        #close box
        elif self.item_text == "Lock":
            servo_data = String()
            servo_data.data = self.item_text
            print(servo_data.data)
            
            self.item_text = "Unlock"
            self.toolButton.setText("Retrieve Item")
            
            pub_servo.publish(servo_data)

    def go_page6(self):
        global popup_new
        popup_new = 0
        msg = String()
        msg.data = 'home'
        pub_home.publish(msg)
        widget.setCurrentIndex(widget.currentIndex()+1)
        

class Eyes(QMainWindow):
    def __init__(self):
        super(Eyes, self).__init__()
        loadUi("/home/tjcc/catkin_ws/src/user_interface/src/GOPHREyes.ui", self)
        
        # move on passed this page when path is complete
        self.home_base = HomeBase(self)
        self.home_base.start()
        self.home_base.success_signal_base.connect(self.go_page1)
        
    def go_page1(self):
        if(widget.currentIndex() == 5):
            widget.setCurrentIndex(widget.currentIndex()-5)



if __name__ == "__main__":
    # * run loop
    # Driver Code
    app = QApplication(sys.argv)
    widget=QtWidgets.QStackedWidget()
    #widget.setFixedHeight(500)
    #widget.setFixedWidth(900)
    #widget.setFixedHeight(600)
    #widget.setFixedWidth(1024)
    widget.showFullScreen()
    widget.setWindowIcon(QtGui.QIcon('/home/tjcc/catkin_ws/src/user_interface/src/Hospital-Icon.ico'))
    # get icon from windows pc for user interface
    widget.setWindowTitle("Hospital App")
    
    # Pages
    page1=Authorize()
    widget.addWidget(page1)
    page2=Instruct()
    widget.addWidget(page2)
    page3=Traveling()
    widget.addWidget(page3)
    page4=SecondAuthorize()
    widget.addWidget(page4)
    page5=Retrieve()
    widget.addWidget(page5)
    page6=Eyes()
    widget.addWidget(page6)
    
    # Bring up window
    widget.show()
    #ui.show()
    rospy.init_node("user_interface") # node name
    pub = rospy.Publisher("/room_number", Int16, queue_size=10) # communicates with send_waypoints.py
    pub_servo = rospy.Publisher('servo_lock', String, queue_size=10) # communicate with servo
    pub_home = rospy.Publisher('uart_home', String, queue_size=10) # communicate with uart home
    

    try:
        sys.exit(app.exec_()) # acts like rospy.spin()
    except KeyboardInterrupt:
        print("Exiting")
        sys.exit(0)
