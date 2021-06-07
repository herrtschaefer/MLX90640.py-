# Generator for Training data by Tim Schäfer

import numpy as np
from MLX90640 import MLX90640 as MLX 
from picamera import PiCamera
import time
from datetime import datetime
import os

#create folders to store data
try:
    os.mkdir('./norm')
except:
    print("ERROR while creating dir norm")

try:
    os.mkdir('./temp')
except:
    print("ERROR while creating dir temp")

try:
    os.mkdir('./raw')
except:
    print("ERROR while creating dir raw")


try:
    os.mkdir('./jpg')
except:
    print("ERROR while creating dir jpg")
    
#setup 
sensor = MLX()
camera = PiCamera()
camera.exposure_mode ='night'
multip = 2 #set resolution of rgb images
camera.resolution = (320*multip,240*multip)
print(camera.exposure_mode)

#to calculate the temperature
TA_SHIFT = 8     #default ambient temp shift for mlx in air
EMISSIVITY = 0.7
TA = 0.0
TR = 0.0

def timestamp():
    return str(datetime.now().strftime("%Y%m%d-%H%M%S"))

#capture and store temperature and RGB data 
def getImage(time):
    MED = 0.0
    while MED == 0.0:
        
        sd = sensor.getFrameData()
        Tr = sensor.getTa() - TA_SHIFT #radiated temp
        To = sensor.CalculateTo(EMISSIVITY,Tr) #temp image
        TO = np.array(To)
        TO = np.reshape(TO,(24,32))
        TO = np.flip(TO,1)
        TN = TO
        TN = np.interp(TN,(TN.min(),TN.max()),(0,1)) #normalize image
        MED = sensor.GetMedian(To) # to validate image data
                 
    RGB(time)
    np.save('./norm/'+time,TN)
    np.save('./temp/'+time,TO)
    np.save('./raw/'+time,np.array(sd))
    

def RGB(time):
    camera.capture('./jpg/'+time+'.jpg')


    
############---generator----###############    
while True:
    
    name = timestamp()
    getImage(name)
    print("img: " + name +  " : ")
    time.sleep(1)

    