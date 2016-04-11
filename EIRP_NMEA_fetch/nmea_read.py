#! /usr/bin/python
# python to read temperature, photo resistor and IMU's LPS data
# 2015/09/21
# Ren Ye

__author__ = 'RE0003YE'

import time
import re
import pynmea2 as nmea
import serial # pySerial for serial data
import numpy as np # for floating point maths
import matplotlib.pyplot as plt # for plot
from drawnow import * # for rendered plot

##### read data from serial port example #####
##arduinoSerialData = serial.Serial('com14', 9600) # need to check it
##
##while (True):
##	if (arduinoSerialData.inWaiting()>0):
##		myData = arduinoSerialData.readline();
##		print(myData)


def readnmea_file(filename):
    f = open(filename, mode='r', encoding='ascii')
    reader = nmea.NMEAStreamReader(f)

    while 1:
        for msg in reader.next():
            print(msg)
            print(msg.sentence_type)
            print(msg.talker)
            #if msg.sentence_type == 'GGA':
            #    print("{0:2.4f}, {1:2.4f}".format(msg.latitude, msg.longitude))
            #elif msg.sentence_type == 'VTG':
            #    print("{0:2.4f}, {1:2.4f}".format(msg.fields[0],msg.fields[1]))



def readnmea_serial(port,baud):
    arduinoSerialData = serial.Serial(port, baud) # open serial connection
##
    while (True):
        if (arduinoSerialData.inWaiting()>0):
            myData = arduinoSerialData.readline() # read the data
            myData = myData.decode('ascii') # convert byte to string, Python3
            #for msg in reader.next(myData):
            print(myData) # for debug purpose

            if(re.search(r'^\$', myData)):
                try:
                    msg = nmea.parse(myData)
                except nmea.ChecksumError:
                    continue

                print(msg.sentence_type)
                print(msg.talker)

#file=open("D:\\EIRP_work\\python\\nmea0183\\data\\airmar_150wx.txt", mode='r')
if __name__ == '__main__':
    port = 'com14'
    baud = 9600
    readnmea_serial(port, baud)


# analogtempv = []
# analogtempc = []
# analogphotov = []
# IMUtempc = []
# IMUbaro = []

# arduinoData = serial.Serial('com14', 9600) #Creating our serial object named arduinoData
# plt.ion() #Tell matplotlib you want interactive mode to plot live data
# cnt=0

# # def makeFig(): #Create a function that makes our desired plot
# #     plt.ylim(20,40)                                 #Set y min and max values
# #     plt.title('Pololu AtlIMU V4 LPS Data')      #Plot the title
# #     plt.grid(True)                                  #Turn the grid on
# #     plt.ylabel('Temp C')                            #Set ylabels
# #     plt.plot(IMUtempc, 'ro-', label='Degrees C')       #plot the temperature
# #     plt.legend(loc='upper left')                    #plot the legend
# #     plt2=plt.twinx()                                #Create a second y axis
# #     plt.ylim(1008,1012)                           #Set limits of second y axis- adjust to readings you are getting
# #     plt2.plot(IMUbaro, 'b^-', label='Pressure (hPa)') #plot pressure data
# #     plt2.set_ylabel('Pressrue (hPa)')                    #label second y axis
# #     plt2.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
# #     plt2.legend(loc='upper right')                  #plot the legend
    
# def makesubFig(): #Create a function that makes our desired plot, with subfigure
#     plt.subplot(3,1,1)
#     plt.ylim(20,40)                                 #Set y min and max values
#     plt.title('Pololu AtlIMU V4 LPS Data')      #Plot the title
#     plt.grid(True)                                  #Turn the grid on
#     plt.ylabel('Temp C')                            #Set ylabels
#     plt.plot(IMUtempc, 'ro-', label='IMU Degrees C')       #plot the IMU temperature
#     plt.plot(analogtempc, 'b^-', label='Analog Degrees C')       #plot the Analog temperature
#     plt.legend(loc='upper left')                    #plot the legend

#     plt.subplot(3,1,2)
#     plt.ylim(1008,1012)                           #Set limits of second y axis- adjust to readings you are getting
#     plt.plot(IMUbaro, 'b^-', label='Pressure (hPa)') #plot pressure data
#     plt.ylabel('Pressrue (hPa)')                    #label second y axis
#     plt.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
#     plt.legend(loc='upper right')                  #plot the legend

#     plt.subplot(3,1,3)
#     plt.ylim(0,5)
#     plt.plot(analogphotov, 'ro-', label='Analog Photo V')
#     plt.plot(analogtempv, 'b^-', label='Analog Temperature V')
#     plt.grid(True)
#     plt.ylabel('Voltage')
#     plt.legend(loc='upper left')


# while True: # While loop that loops forever
#     while (arduinoData.inWaiting()==0): #Wait here until there is data
#         pass #do nothing
#     arduinoBytes = arduinoData.readline() #read the line of text from the serial port
#     arduinoString = arduinoBytes.decode('ascii')
#     print(arduinoString)
#     dataArray = arduinoString.split(',')   #Split it into an array called dataArray
#     analogtemperaturev = float( dataArray[0])            #Convert element 1 to analog temperature voltage
#     analogtemperaturec = float( dataArray[1])            #Convert element 2 to analog temperature c
#     analogphotoresistancev = float( dataArray[2])            #Convert element 3 to photo resistor voltage
#     IMUtemperature = float( dataArray[3])            #Convert element 4 to IMU temperature c
#     IMUpressure = float( dataArray[4])            #Convert element 5 to IM pressure
#     IMUtempc.append(IMUtemperature)                     #Build our tempF array by appending temp readings
#     IMUbaro.append(IMUpressure)                     #Building our pressure array by appending P readings
#     analogtempv.append(analogtemperaturev)                     #Building our pressure array by appending P readings
#     analogtempc.append(analogtemperaturec)                     #Building our pressure array by appending P readings
#     analogphotov.append(analogphotoresistancev)                     #Building our pressure array by appending P readings
#     drawnow(makesubFig)                       #Call drawnow to update our live graph
#     plt.pause(.000001)                     #Pause Briefly. Important to keep drawnow from crashing
#     cnt=cnt+1
#     if(cnt>50):                            #If you have 50 or more points, delete the first one from the array
#         IMUtempc.pop(0)                       #This allows us to just see the last 50 data points
#         IMUbaro.pop(0)
#         analogtempv.pop(0)
#         analogtempc.pop(0)
#         analogphotov.pop(0)
