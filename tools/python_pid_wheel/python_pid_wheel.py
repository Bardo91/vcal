import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import fastcom.Subscriber
import fastcom.Publisher
import serial
import threading
import time
import struct

AppRunning = True

class Radiodemo(QWidget):
    updateParam = pyqtSignal()
    def __init__(self, _uavAddr, _portX, _portY, _portZ, _portPubX, _portPubY, _portPubZ, parent=None):
        super(Radiodemo, self).__init__(parent)
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)
        self.setWindowTitle("PID WHEEL")

        displayGroup = QGroupBox("Display")
        displayLayout = QHBoxLayout()
        displayGroup.setLayout(displayLayout)
        mainLayout.addWidget(displayGroup)
        self.display = QLineEdit()
        self.display.setReadOnly(True)
        self.display.setFixedWidth(200)
        displayLayout.addWidget(self.display)

        
        resolutionGroup = QGroupBox("resolution")
        resolutionLayout = QHBoxLayout()
        resolutionGroup.setLayout(resolutionLayout)
        mainLayout.addWidget(resolutionGroup)
        self.resolution = QLineEdit()
        self.resolution.setText(str(0.01))
        self.resolution.setFixedWidth(200)
        resolutionLayout.addWidget(self.resolution)


        pidGroup = QGroupBox("PID controller")
        mainLayout.addWidget(pidGroup)
        pidLayout = QHBoxLayout()
        pidGroup.setLayout(pidLayout)
        self.b_pid_x = QRadioButton("PID_X")
        pidLayout.addWidget(self.b_pid_x)
        self.b_pid_y = QRadioButton("PID_Y")
        pidLayout.addWidget(self.b_pid_y)
        self.b_pid_z = QRadioButton("PID_Z")
        pidLayout.addWidget(self.b_pid_z)

        paramGroup = QGroupBox("Parameter")
        mainLayout.addWidget(paramGroup)
        pidParam = QHBoxLayout()
        paramGroup.setLayout(pidParam)
        self.b_pid_kp = QRadioButton("kp")
        pidParam.addWidget(self.b_pid_kp)
        self.b_pid_ki = QRadioButton("ki")
        pidParam.addWidget(self.b_pid_ki)
        self.b_pid_kd = QRadioButton("kd")
        pidParam.addWidget(self.b_pid_kd)
        self.b_pid_sat = QRadioButton("sat")
        pidParam.addWidget(self.b_pid_sat)
        self.b_pid_wind = QRadioButton("wind")
        pidParam.addWidget(self.b_pid_wind)

        self.pid_params_x = [0,0,0,0,0]
        self.pid_params_y = [0,0,0,0,0]
        self.pid_params_z = [0,0,0,0,0]

        self.subPidX = fastcom.Subscriber.Subscriber(_uavAddr,_portX)
        self.subPidX.appendCallback(self.callback_X)
        
        self.subPidY = fastcom.Subscriber.Subscriber(_uavAddr,_portY)
        self.subPidY.appendCallback(self.callback_Y)
        
        self.subPidZ = fastcom.Subscriber.Subscriber(_uavAddr,_portZ)
        self.subPidZ.appendCallback(self.callback_Z)

        self.updateParam.connect(self.updateParamCallback)

        self.updateParamsThread = threading.Thread(target=self.__guiUpdaterTimer)
        self.updateParamsThread.start()


        self.pubPidX = fastcom.Publisher.Publisher(_portPubX)
        self.pubPidY = fastcom.Publisher.Publisher(_portPubY)
        self.pubPidZ = fastcom.Publisher.Publisher(_portPubZ)

        self.serialReadThread = threading.Thread(target=self.__serialCallback)
        self.serialReadThread.start()


    def __guiUpdaterTimer(self):
        while True:
            self.updateParam.emit()
            time.sleep(0.03)

    def __serialCallback(self):
        ser = serial.Serial('/dev/ttyUSB0')  # open serial port
        while True:
            line = ser.readline()
            inc = float(line)*float(self.resolution.text())

            newParams = self.pid_params_x
            if(self.b_pid_x.isChecked()):
                newParams = self.pid_params_x
            elif(self.b_pid_y.isChecked()):
                newParams = self.pid_params_y
            elif(self.b_pid_z.isChecked()):
                newParams = self.pid_params_z
            
            if(self.b_pid_kp.isChecked()):
                newParams[0] = newParams[0] + inc
            elif(self.b_pid_ki.isChecked()):
                newParams[1] = newParams[1] + inc
            elif(self.b_pid_kd.isChecked()):
                newParams[2] = newParams[2] + inc
            elif(self.b_pid_sat.isChecked()):
                newParams[3] = newParams[3] + inc
            elif(self.b_pid_wind.isChecked()):
                newParams[3] = newParams[4] + inc

            if(self.b_pid_x.isChecked()):
                self.pubPidX.publish(struct.pack("fffff", newParams[0], newParams[1], newParams[2], newParams[3], newParams[4]))
            elif(self.b_pid_y.isChecked()):
                self.pubPidY.publish(struct.pack("fffff", newParams[0], newParams[1], newParams[2], newParams[3], newParams[4]))
            elif(self.b_pid_z.isChecked()):
                self.pubPidZ.publish(struct.pack("fffff", newParams[0], newParams[1], newParams[2], newParams[3], newParams[4]))


    def updateParamCallback(self):
        if(self.b_pid_x.isChecked()):
            if(self.b_pid_kp.isChecked()):
                self.display.setText(str(self.pid_params_x[0]))
            elif(self.b_pid_ki.isChecked()):
                self.display.setText(str(self.pid_params_x[1]))
            elif(self.b_pid_kd.isChecked()):
                self.display.setText(str(self.pid_params_x[2]))
            elif(self.b_pid_sat.isChecked()):
                self.display.setText(str(self.pid_params_x[3]))
            elif(self.b_pid_wind.isChecked()):
                self.display.setText(str(self.pid_params_x[4]))
        elif(self.b_pid_y.isChecked()):
            if(self.b_pid_kp.isChecked()):
                self.display.setText(str(self.pid_params_y[0]))
            elif(self.b_pid_ki.isChecked()):
                self.display.setText(str(self.pid_params_y[1]))
            elif(self.b_pid_kd.isChecked()):
                self.display.setText(str(self.pid_params_y[2]))
            elif(self.b_pid_sat.isChecked()):
                self.display.setText(str(self.pid_params_y[3]))
            elif(self.b_pid_wind.isChecked()):
                self.display.setText(str(self.pid_params_y[4]))
        elif(self.b_pid_z.isChecked()):
            if(self.b_pid_kp.isChecked()):
                self.display.setText(str(self.pid_params_z[0]))
            elif(self.b_pid_ki.isChecked()):
                self.display.setText(str(self.pid_params_z[1]))
            elif(self.b_pid_kd.isChecked()):
                self.display.setText(str(self.pid_params_z[2]))
            elif(self.b_pid_sat.isChecked()):
                self.display.setText(str(self.pid_params_z[3]))
            elif(self.b_pid_wind.isChecked()):
                self.display.setText(str(self.pid_params_z[4]))
        

    def callback_X(self, data):
        self.pid_params_x = list(struct.unpack("fffff", data))
    
    def callback_Y(self, data):
        self.pid_params_y = list(struct.unpack("fffff", data))
    
    def callback_Z(self, data):
        self.pid_params_z = list(struct.unpack("fffff", data))

            

def main():
    app = QApplication([])
    ex = Radiodemo(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]))
    ex.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
