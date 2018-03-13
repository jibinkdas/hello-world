import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera

##camera = PiCamera()
##rawCapture = PiRGBArray(camera)

with picamera.PiCamera() as camera:
    camera.start_preview()
    camera.hflip = True
    try:
        for i, filename in enumerate(camera.capture_continuous('image{counter:02d}.jpg')):
            print(filename)
            img = cv2.imread(filename)
            cv2.imshow('Test',img)
            time.sleep(1)
            
            if i == 2:
                
                break
    finally:
        camera.stop_preview()






r1=0.0
c=0
baudRate = 9600 # change this to the required value
arduinoPort = serial.Serial('/dev/ttyACM0',baudRate, timeout=0)
arduinoPort.flush()
s=0



#image1 = np.zeros((640,480,3),np.uint8)
image = cv2.imread("image02.jpg") # change this to the actual filename
##image = cv2.imread("imagescale03.jpg")
#image2 = cv2.resize(image2,(640,480))
#image = np.hstack((image2,image2))
                 
hc,wc,c = image.shape
font = cv2.FONT_HERSHEY_COMPLEX_SMALL
cv2.namedWindow("Height and Stagger")
points=[]
calib_distance = 645 # in mm
obj_distance_mm = 100 
obj_distance_pixels = 1
focal_length_pixels = 1193.25

focal_length_mm = 3.0
pixel_per_mm = wc*400/1366
object_real_world_mm = 32
object_image_sensor_mm = 1
center = [int(wc/2),int(hc/2)]
print(wc)

def calibrate(action, x, y, flags, userdata):
    if action == cv2.EVENT_LBUTTONDBLCLK:
        point = [x, y]
        points.append(point)
        cv2.putText(image, str(point), (250, 250), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
dist='dist='
distdata=''
lon='longitude  :'
longitude=''
lat='latitude :'
longitude=''
angle='orientation='
angledata=''
def read_sensorData():

    latnum=0.0
    lonnum=0.0
    latitude=0.0
    longitude=0.0
    image1 = np.zeros((640,480,3),np.uint8)
    data= arduinoPort.readline().decode()
    #print(data)
    if data == 'start\n':            
        dist= data.strip('\n')
        print(dist)
        data= arduinoPort.readline().decode()
        with picamera.PiCamera() as camera:
            camera.start_preview()
            try:
                for i, filename in enumerate(camera.capture_continuous('image{counter:02d}.jpg')):
                    print(filename)
                    img = cv2.imread(filename)
                    cv2.imshow('Test',img)
                    time.sleep(1)
                    
                    if i == 2:
                        
                        break
            finally:
                camera.stop_preview()
                image = cv2.imread("image02.jpg")                
##                image = cv2.imread("imagescale03.jpg")
                cv2.destroyWindow("Height and Stagger")
                cv2.imshow("Height and Stagger2",image)
    if data == 'orientation=\n':
        angle= data.strip('\n')
        print(angle)
        data= arduinoPort.readline().decode()
        angledata= data.strip('\n')
        print(angledata)
        cv2.putText(image1,"orientation = "+angledata+"degree",(10, 105),font,1,(0, 255, 255),1,cv2.LINE_AA)
        data= arduinoPort.readline().decode()           
    if data == 'Latitude  :\n':
        lat= data.strip('\n')
        print(lat)
        cv2.putText(image1,"latitude = ",(10, 35),font,1,(0, 255, 255),1,cv2.LINE_AA)
        data= arduinoPort.readline().decode()
        latdata= data.strip('\n')
        print(data)
        if len(latdata)>5:
            latnum=latdata
            d=int(latnum[0:2])
            d1=float(latnum[2:9])
            d1=d1/60
            latitude=d+d1
            latitude=np.round(latitude,6)
            latitude=str(latitude)
            cv2.putText(image1,"latitude = "+latitude+"degree",(10, 35),font,1,(0, 255, 255),1,cv2.LINE_AA)
            data= arduinoPort.readline().decode()       
            if data == 'Longitude :\n':
                lon= data.strip('\n')
                print(lon)
                cv2.putText(image1,"longitude = ",(10, 70),font,1,(0, 255, 255),1,cv2.LINE_AA)
                data= arduinoPort.readline().decode()
                londata= data.strip('\n')
                print(londata)
                if len(londata)>5:
                    lonnum=londata
                    d=int(lonnum[0:3])
                    d1=float(lonnum[3:10])
                    d1=d1/60
                    longitude=d+d1
                    longitude=np.round(longitude,6)
                    longitude=str(longitude)
                    cv2.putText(image1,"longitude = "+longitude+"degree",(10, 70),font,1,(0, 255, 255),1,cv2.LINE_AA)
                    data= arduinoPort.readline().decode()
                    if data == 'orientation=\n':
                        angle= data.strip('\n')
                        print(angle)
                        data= arduinoPort.readline().decode()
                        angledata= data.strip('\n')
                        print(angledata)
                        cv2.putText(image1,"orientation = "+angledata+"degree",(10, 105),font,1,(0, 255, 255),1,cv2.LINE_AA)
                else:
                    longitude=""
                    cv2.putText(image1,"longitude = "+longitude+"degree",(10, 70),font,1,(0, 255, 255),1,cv2.LINE_AA)
        else:
            latitude=""
            longitude=""
            cv2.putText(image1,"latitude = "+latitude+"degree",(10, 35),font,1,(0, 255, 255),1,cv2.LINE_AA)
            cv2.putText(image1,"longitude = "+longitude+"degree",(10, 70),font,1,(0, 255, 255),1,cv2.LINE_AA)

    else:
        #print("latitude not find")
        longitude=""
        latitude=""
        cv2.putText(image1,"latitude = "+latitude+"degree",(10, 35),font,1,(0, 255, 255),1,cv2.LINE_AA)
        cv2.putText(image1,"longitude = "+longitude+"degree",(10, 70),font,1,(0, 255, 255),1,cv2.LINE_AA)


    cv2.imshow("orientation and location",image1)
    if cv2.waitKey(20) & 0xFF == 27:
        print("")
    time.sleep(.2)

    
cv2.setMouseCallback("Height and Stagger",calibrate)
cv2.imshow("Height and Stagger",image)


while(1):
    
    read_sensorData()
    cv2.imshow("Height and Stagger",image)
    
    
    
   
    if len(points) > 1:
        
        distance = np.sqrt((points[0][0]-points[1][0])**2 + (points[0][1]-points[1][1])**2)
        #object_image_sensor_mm = distance / pixel_per_mm
        obj_distance_pixels = distance
        scale = distance/obj_distance_mm
        #focal_length_pixels = calib_distance*obj_distance_pixels/obj_distance_mm
        #cv2.putText(image,str(np.round(distance,2)), (10, 50), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        #distance_mm = object_real_world_mm * focal_length_mm / object_image_sensor_mm
        distance_mm = focal_length_pixels * 590 / distance
##        distance_mm = distance_mm - (distance_mm * 0.11)


        cv2.putText(image,"scale = "+str(np.round(scale,3))+"pixels/mm",(10, 35),font,1,(0, 255, 255),1,cv2.LINE_AA)
        wire_center_x = points[1][0] + int((points[0][0]- points[1][0]) / 2)
        wire_center_y = points[1][1] + int((points[0][1] - points[1][1]) / 2)


        stagger_mm = np.sqrt((wire_center_x - center[0])**2 + (wire_center_y - center[1])**2)/1.8
        cv2.putText(image,"stagger = "+str(np.round(stagger_mm,2))+"mm",(10, 70),font,1,(0, 0, 255),1,cv2.LINE_AA)
        cv2.line(image, (center[0], center[1]), (wire_center_x, wire_center_y), (0, 0, 255), 1)
##        points=[]
##        print (wire_center_x)
##        print(wire_center_y)
##        print (stagger_mm)
        print("focallengthinpixels = "+ str(focal_length_pixels))
        print(points)
        cv2.imshow("Height and Stagger",image)
        if cv2.waitKey(20) & 0xFF == 27:
            break
        time.sleep(.2)

cv2.destroyAllWindows()



