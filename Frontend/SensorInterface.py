import sys
from PyQt5 import QtCore, QtGui, QtWidgets, Qt
from PyQt5 import uic
import communicator as com
import numpy as np
import time
import csv
import pyqtgraph as pg

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi("widget.ui", self)
        
        self.thePen = pg.mkPen(color='k', style=QtCore.Qt.SolidLine, width = 2)
        self.axisPen = pg.mkPen(color='k', style=QtCore.Qt.SolidLine, width = 2)
        self.label_style = {'font-size': '10pt'}
        self.graphWidget.setXRange(-10, 0, padding=0.03)
        self.graphWidget.getAxis('left').setTextPen('k')
        self.graphWidget.getAxis('bottom').setTextPen('k')
        self.graphWidget.getAxis('left').setPen(self.axisPen)
        self.graphWidget.getAxis('bottom').setPen(self.axisPen)
        self.graphWidget.setLabel('left', "R (a.u.)", **self.label_style)
        self.graphWidget.setLabel('bottom', "t (s)", **self.label_style)
        self.graphWidget.getAxis('bottom').showLabel(show = True)
        self.graphWidget.getAxis('right').setStyle(tickLength = 0)
        self.graphWidget.getAxis('top').setStyle(tickLength = 0)
        self.graphWidget.showAxis('top')
        self.graphWidget.showAxis('right')
        self.graphWidget.getAxis('top').setPen(self.axisPen)
        self.graphWidget.getAxis('right').setPen(self.axisPen)
        self.graphWidget.getAxis('right').setTextPen(Qt.QColor(0,0,0,0))
        self.graphWidget.getAxis('top').setTextPen(Qt.QColor(0,0,0,0))
        
        self.xvals = []
        self.toPlotA = []
        self.dataCurve1 = self.graphWidget.plot(self.xvals, self.toPlotA, pen = self.thePen)
        
        
        self.measureButton.clicked.connect(self.measureButtonClicked)
        self.backgroundButton.clicked.connect(self.backgroundButtonClicked)
        self.recordButton.clicked.connect(self.recordButtonClicked)
        self.connectButton.clicked.connect(self.checkPort)
        self.dialogueBox.setText("Connect to a device to start measurements")
        self.baseline = 0
        self.backgroundBrowser.setText(str(self.baseline))  
        self.resultIndex = 0
        
    def measureButtonClicked(self):
        samples = self.samplesBox.value()
        readings = np.array([])
        try:
            for i in range(0,samples):
                reading = com.ReadDevice(self.port).split()
                newVal = float(reading[self.resultIndex])
                readings = np.append(readings, newVal)
                progress = int(100*(float(readings.size)/float(samples)))
                self.progressBar.setValue(progress)
                app.processEvents()
             
            measurement = readings.mean()
            subbed = measurement - self.baseline
            stdev = readings.std()
            
            if self.checkBoxSubtract.isChecked():
                outstring = "{:.5f} +/- {:.5f}".format(subbed, stdev)
            else:
                outstring = "{:.5f} +/- {:.5f}".format(measurement, stdev)
                
            self.measureBrowser.setText(outstring)
            
            self.dialogueBox.setText("Measurement done")
                
        except:
            self.dialogueBox.setText("Failed! Have you connected?")

    def backgroundButtonClicked(self):
        #print ("background")
        
        samples = self.samplesBox.value()
        readings = np.array([])
        
        try:
            for i in range(0,samples):
                reading = com.ReadDevice(self.port).split()
                newVal = float(reading[self.resultIndex])
                readings = np.append(readings, newVal)
                progress = int(100*(float(readings.size)/float(samples)))
                self.progressBar.setValue(progress)
                app.processEvents()
                
            self.baseline = readings.mean()
            stdev = readings.std()
            
            self.dialogueBox.setText("Baseline done")
            outstring = "{:.5f} +/- {:.5f}".format(self.baseline, stdev)
            self.backgroundBrowser.setText(outstring) 
            
        except:
            self.dialogueBox.setText("Failed! Have you connected?")
        

    def recordButtonClicked(self):
        if self.recordButton.isChecked() == 0:
            self.recordButton.setText("Start Continuous")
            self.dialogueBox.setText("Continuous measurement stopped")
            self.checkBoxRecord.setDisabled(False)
            

        else:
            self.toPlotA = np.array([])
            self.checkBoxRecord.setDisabled(True)
            if self.checkBoxRecord.isChecked():
                
                filename = self.filenameEdit.text()
                f = open(filename, 'w', newline='')
                writer = csv.writer(f)

            self.recordButton.setText("Stop") #changes the button. this is the go bit
            self.dialogueBox.setText("Continuous measurement started!")
            while self.recordButton.isChecked() == 1:
                
                plotRange = self.spinBox.value()
                
                self.graphWidget.setXRange(-1*plotRange, 0, padding=0.03)
                
                reading = com.ReadDevice(self.port).split()
                measured = float(reading[self.resultIndex]) 
                subbed = measured - self.baseline
                row = [measured, subbed]
                
                if self.checkBoxSubtract_2.isChecked():                    
                    self.toPlotA = np.append(self.toPlotA, subbed)
                    self.graphWidget.setLabel('left', "A - A<sub>0</sub>", **self.label_style)
                else:
                    self.toPlotA = np.append(self.toPlotA, measured)
                    self.graphWidget.setLabel('left', "A", **self.label_style)
                    
                plotLength = self.toPlotA.size
                
                
                self.xvals = np.linspace(-0.1*(plotLength-1), 0, plotLength)
                
                self.dataCurve1.setData(self.xvals, self.toPlotA)

                if self.checkBoxRecord.isChecked():    
                    writer.writerow(row)
                    
                app.processEvents()
                
                
            if self.checkBoxRecord.isChecked():     
                f.close()

    
    def checkPort(self):
        self.port = str(self.portSelect.text())
        check = com.isOpen(self.port)    #port checking function in communicator
        if check=="ready":
            self.dialogueBox.setText("Ready!")
        elif check=="not_ready":
            self.dialogueBox.setText("Not ready. Wait for device, or check correct port is selected.")
        else:
            self.dialogueBox.setText("Not connected!")
        

pg.setConfigOption('background', Qt.QColor(0,0,0,0))
pg.setConfigOption('foreground', 'k')
pg.setConfigOptions(antialias=True)

app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()
