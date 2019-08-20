from PyQt5 import QtCore, QtGui,QtWidgets
from PyQt5.QtWidgets import QWidget, QDesktopWidget, QApplication,QMainWindow,QMessageBox
from m600_ui import Ui_Frame as ui
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from serial_test.msg import gimCtr
from serial_test.msg import emergency
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import rospy
import sys

class M600_UI(QWidget):
	displayLoopSig = QtCore.pyqtSignal(dict)
	def __init__(self):
		super(M600_UI,self).__init__()	
		self._ui = ui()
		self._ui.setupUi(self)
		self.setWindowTitle('M600_beta')
		self.init_button()
		self.init_pubSub()
		self.displayLoop()
		
		
	def init_button(self):
		self._ui.pushButton.clicked.connect(lambda:self.pubVelCtr(2))   #left
		self._ui.pushButton_2.clicked.connect(lambda:self.pubVelCtr(-2))  	 #right
		self._ui.pushButton_3.clicked.connect(lambda:self.pubVelCtr(1))  	 #forward
		self._ui.pushButton_4.clicked.connect(lambda:self.pubVelCtr(4))  	 #yaw_left
		self._ui.pushButton_5.clicked.connect(lambda:self.pubVelCtr(-1))  	 #backward
		self._ui.pushButton_6.clicked.connect(lambda:self.pubVelCtr(-4))  	 #yaw_right
		self._ui.pushButton_7.clicked.connect(lambda:self.pubVelCtr(3))  	 #up
		self._ui.pushButton_8.clicked.connect(lambda:self.pubVelCtr(-3))  	 #down
		self._ui.pushButton_9.clicked.connect(lambda:self.pubEmCtr(1))	 #takeOff
		self._ui.pushButton_10.clicked.connect(lambda:self.pubEmCtr(3))	 #home
		#self._ui.pushButton_11.clicked.connect(lambda:self.pubEmCtr(2))	 #land
		self._ui.pushButton_12.clicked.connect(lambda:self.pubGimCtr(-2))	 #gim_yawL
		self._ui.pushButton_13.clicked.connect(lambda:self.pubGimCtr(-1))		 #gimPD
		self._ui.pushButton_14.clicked.connect(lambda:self.pubGimCtr(2))		 #gim_yawR
		self._ui.pushButton_15.clicked.connect(lambda:self.pubGimCtr(0))		 #reset
		self._ui.pushButton_16.clicked.connect(lambda:self.pubGimCtr(1))		 #gimPU
		self._ui.pushButton_23.clicked.connect(lambda:self.pubGimCtr(3))		 #zoomOut
		self._ui.pushButton_24.clicked.connect(lambda:self.pubGimCtr(-3))		 #zoomIn
		self._ui.pushButton_25.clicked.connect(lambda:self.pubGimCtr(4))		#sor
		self._ui.pushButton_26.clicked.connect(lambda:self.pubGimCtr(5))		#sos
		self._ui.pushButton_17.clicked.connect(lambda:self.pubAuthority(1))		#en_auth
		self._ui.pushButton_18.clicked.connect(lambda:self.pubAuthority(2))		#dis_auth
	
        
		self._ui.pushButton.setAutoRepeat(True)  
		self._ui.pushButton_2.setAutoRepeat(True)	 #right
		self._ui.pushButton_3.setAutoRepeat(True)  	 #forward
		self._ui.pushButton_4.setAutoRepeat(True) 	 #yaw_left
		self._ui.pushButton_5.setAutoRepeat(True) 	 #backward
		self._ui.pushButton_6.setAutoRepeat(True) 	 #yaw_right
		self._ui.pushButton_7.setAutoRepeat(True) 	 #up
		self._ui.pushButton_8.setAutoRepeat(True)	 #down
		
		
			
   
	def init_pubSub(self):
		self.velCtr = Twist()
		self.gimCtr = gimCtr()
		self.emCtr = emergency()
		self.authCtr = UInt8()
			
		self.gimPub = rospy.Publisher("/m600_gimCtr",gimCtr,queue_size = 10)
		self.velPub = rospy.Publisher("/m600_velCtr",Twist,queue_size = 10)
		self.emPub = rospy.Publisher("/m600_emergency",emergency,queue_size = 10)
		self.authoPub = rospy.Publisher("/m600_authority",UInt8,queue_size = 10)


		self.gpsSUb = rospy.Subscriber("m600_gps",NavSatFix,self.gpsCallback,queue_size = 10)
		self.attSUb = rospy.Subscriber("m600_attitude",Vector3,self.attCallback,queue_size = 10)
		self.odomSUb = rospy.Subscriber("m600_odom",Odometry,self.odomCallback,queue_size = 10)



	def gpsCallback(self,gpsMsg):
		self.gps_msg = gpsMsg

	def attCallback(self,attMsg):
		self.att_msg = attMsg

	def odomCallback(self,odomMsg):
		self.odom_msg = odomMsg
	
	
  	def displayLoop(self):
		self.infoDict = {}
		self.displayLoopSig.connect(self.displayInfo)
		self.displayTimer = rospy.Timer(rospy.Duration(1000/1000.0),self.updateInfo)

	def updateInfo(self,event):
		self.infoDict["status"] = 0
		self.infoDict["battery"] = 85
		self.infoDict["gpsSinal"] = 5
		self.infoDict["lat"] = self.gps_msg.latitude
		self.infoDict["lnt"] = self.gps_msg.longitude
		self.infoDict["alt"] = self.gps_msg.altitude
		self.infoDict["velX"] = self.odom_msg.twist.twist.linear.x
		self.infoDict["velY"] = self.odom_msg.twist.twist.linear.y
		self.infoDict["velZ"] = self.odom_msg.twist.twist.linear.z
		self.infoDict["pitch"] = self.att_msg.x
		self.infoDict["roll"] =  self.att_msg.y
		self.infoDict["yaw"] =  self.att_msg.z
		self.displayLoopSig.emit(self.infoDict)
		

	def displayInfo(self,uavInfo):
		self._ui.label_10.setText("status: "+str(uavInfo["status"]))
        	self._ui.label_11.setText("battery: "+str(uavInfo["battery"]))
        	self._ui.label_12.setText("gpsSinal: "+str(uavInfo["gpsSinal"]))
		self._ui.label_13.setText("lat: "+str(uavInfo["lat"]))
		self._ui.label_14.setText("lnt: "+str(uavInfo["lnt"]))
		self._ui.label_16.setText("alt: "+str(uavInfo["alt"]))
		self._ui.label.setText("velX: "+str(uavInfo["velX"]))
		self._ui.label_2.setText("velY: "+str(uavInfo["velY"]))
		self._ui.label_3.setText("velZ: "+str(uavInfo["velZ"]))
		self._ui.label_4.setText("pitch: "+str(uavInfo["pitch"]))
		self._ui.label_5.setText("roll: "+str(uavInfo["roll"]))
		self._ui.label_6.setText("yaw: "+str(uavInfo["yaw"]))
	

	def pubVelCtr(self,key):
		if key == 1:
			self.velCtr.linear.x = 2
		elif key == -1:
			self.velCtr.linear.x = -2
		elif key == 2:
			self.velCtr.linear.y = 2
		elif key == -2:
			self.velCtr.linear.y = -2
		elif key == 3:
			self.velCtr.linear.z = 2
		elif key == -3:
			self.velCtr.linear.z = -2
		elif key == 4:
			self.velCtr.angular.z = 2
		elif key == -4:
			self.velCtr.angular.z = -2
		
		self.velPub.publish(self.velCtr)
		self.velCtr.linear.x = 0
		self.velCtr.linear.y = 0
		self.velCtr.linear.z = 0
		self.velCtr.angular.z = 0


	def pubGimCtr(self,key):
		if key == 1:
			self.gimCtr.pry.x = 10
		elif key == -1:
			self.gimCtr.pry.x = -10
		elif key == 2:
			self.gimCtr.pry.z = 10
		elif key == -2:
			self.gimCtr.pry.z = -10
		elif key == 0:
			self.gimCtr.reset.data = 1
		elif key == 3:
			self.gimCtr.mutiple.data = 1
		elif key == -3:
			self.gimCtr.mutiple.data = -1
		elif key == 4:
			self.gimCtr.sor.data = 1
		elif key ==  5:
			self.gimCtr.sos.data = 1
		self.gimPub.publish(self.gimCtr)

		self.gimCtr.pry.x=0
		self.gimCtr.pry.y=0
		self.gimCtr.pry.z=0
		self.gimCtr.mutiple.data=0
		self.gimCtr.reset.data=0
		self.gimCtr.sos.data=0
		self.gimCtr.sor.data=0
		self.gimCtr.setFcus.data=0
		
		

	def pubEmCtr(self,key):
		if key == 1:
			self.emCtr.takeOff.data = 1
		elif key == 2:
			self.emCtr.land.data =1
		elif key == 3:
			self.emCtr.home.data =1
		elif key == 4:
			self.emCtr.hover.data =1
		self.emPub.publish(self.emCtr)

		self.emCtr.takeOff.data =0
		self.emCtr.land.data =0
		self.emCtr.home.data =0
		self.emCtr.hover.data =0



	def pubAuthority(self,key):
		if key == 1:
			self.authCtr.data = 1
		elif key == 2:
			self.authCtr.data = 2
		self.authoPub.publish(self.authCtr)
		self.authCtr.data = 0
		
		
	def closeEvent(self, event):
		reply = QMessageBox.question(self, 'Message', 'You sure to quit?',QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
		if reply == QMessageBox.Yes:
			event.accept()
		else:
			event.ignore()



if __name__=='__main__':
	rospy.init_node('m600_ui')
	app = QApplication(sys.argv)
	display = M600_UI()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('great flying')
	sys.exit(status)	


