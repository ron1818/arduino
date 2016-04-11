#! /usr/bin/python
# python to read temperature, photo resistor and IMU's LPS data
# 2015/09/21
# Ren Ye

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


##### matplotlib example #####
## x=[]
## y=[]
## 
## 
## for i in np.arange(0,2*np.pi,2*np.pi/1000):
##     x.append(i)
##     y.append(np.sin(i))
## 
## plt.plot(x,y, 'b-', linewidth=2)
## plt.grid(True)
## plt.axis([0,2*np.pi,-1.5,1.5])
## plt.title('My Sine Wave')
## plt.xlabel('Time in Seconds')
## plt.ylabel('Sin(t)')
## plt.show()
## 
## x= np.linspace( 0, 2*np.pi, 50)#create your x array
## y= np.sin(x) #create y array
## z= np.cos(x) #create z array
## plt.plot(x,y, 'b-d', linewidth=2, label='sinx') #plot y
## plt.plot(x,z, 'r-o', linewidth=2, label='cosx') #plot z
## plt.grid(True) #display background grid
## plt.axis([0,2*np.pi,-1.5,1.5]) #set range on axis
## plt.title('My Sin and Cos Waves') #chart title
## plt.xlabel('Time in Seconds') #label x axis
## plt.ylabel('My Waves') #label y axis
## plt.legend() #show legend
## plt.show() #show the plot

analogtempv = []
analogtempc = []
analogphotov = []
IMUtempc = []
IMUbaro = []

arduinoData = serial.Serial('com14', 9600) #Creating our serial object named arduinoData
plt.ion() #Tell matplotlib you want interactive mode to plot live data
cnt=0

# def makeFig(): #Create a function that makes our desired plot
#     plt.ylim(20,40)                                 #Set y min and max values
#     plt.title('Pololu AtlIMU V4 LPS Data')      #Plot the title
#     plt.grid(True)                                  #Turn the grid on
#     plt.ylabel('Temp C')                            #Set ylabels
#     plt.plot(IMUtempc, 'ro-', label='Degrees C')       #plot the temperature
#     plt.legend(loc='upper left')                    #plot the legend
#     plt2=plt.twinx()                                #Create a second y axis
#     plt.ylim(1008,1012)                           #Set limits of second y axis- adjust to readings you are getting
#     plt2.plot(IMUbaro, 'b^-', label='Pressure (hPa)') #plot pressure data
#     plt2.set_ylabel('Pressrue (hPa)')                    #label second y axis
#     plt2.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
#     plt2.legend(loc='upper right')                  #plot the legend
    
def makesubFig(): #Create a function that makes our desired plot, with subfigure
    plt.subplot(3,1,1)
    plt.ylim(20,40)                                 #Set y min and max values
    plt.title('Pololu AtlIMU V4 LPS Data')      #Plot the title
    plt.grid(True)                                  #Turn the grid on
    plt.ylabel('Temp C')                            #Set ylabels
    plt.plot(IMUtempc, 'ro-', label='IMU Degrees C')       #plot the IMU temperature
    plt.plot(analogtempc, 'b^-', label='Analog Degrees C')       #plot the Analog temperature
    plt.legend(loc='upper left')                    #plot the legend

    plt.subplot(3,1,2)
    plt.ylim(1008,1012)                           #Set limits of second y axis- adjust to readings you are getting
    plt.plot(IMUbaro, 'b^-', label='Pressure (hPa)') #plot pressure data
    plt.ylabel('Pressrue (hPa)')                    #label second y axis
    plt.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
    plt.legend(loc='upper right')                  #plot the legend

    plt.subplot(3,1,3)
    plt.ylim(0,5)
    plt.plot(analogphotov, 'ro-', label='Analog Photo V')
    plt.plot(analogtempv, 'b^-', label='Analog Temperature V')
    plt.grid(True)
    plt.ylabel('Voltage')
    plt.legend(loc='upper left')


while True: # While loop that loops forever
    while (arduinoData.inWaiting()==0): #Wait here until there is data
        pass #do nothing
    arduinoBytes = arduinoData.readline() #read the line of text from the serial port
    arduinoString = arduinoBytes.decode('ascii')
    print(arduinoString)
    dataArray = arduinoString.split(',')   #Split it into an array called dataArray
    analogtemperaturev = float( dataArray[0])            #Convert element 1 to analog temperature voltage
    analogtemperaturec = float( dataArray[1])            #Convert element 2 to analog temperature c
    analogphotoresistancev = float( dataArray[2])            #Convert element 3 to photo resistor voltage
    IMUtemperature = float( dataArray[3])            #Convert element 4 to IMU temperature c
    IMUpressure = float( dataArray[4])            #Convert element 5 to IM pressure
    IMUtempc.append(IMUtemperature)                     #Build our tempF array by appending temp readings
    IMUbaro.append(IMUpressure)                     #Building our pressure array by appending P readings
    analogtempv.append(analogtemperaturev)                     #Building our pressure array by appending P readings
    analogtempc.append(analogtemperaturec)                     #Building our pressure array by appending P readings
    analogphotov.append(analogphotoresistancev)                     #Building our pressure array by appending P readings
    drawnow(makesubFig)                       #Call drawnow to update our live graph
    plt.pause(.000001)                     #Pause Briefly. Important to keep drawnow from crashing
    cnt=cnt+1
    if(cnt>50):                            #If you have 50 or more points, delete the first one from the array
        IMUtempc.pop(0)                       #This allows us to just see the last 50 data points
        IMUbaro.pop(0)
        analogtempv.pop(0)
        analogtempc.pop(0)
        analogphotov.pop(0)
