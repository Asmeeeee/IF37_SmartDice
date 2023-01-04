from imu import MPU6050
from time import sleep
from machine import Pin, I2C
import time
import math 
Rad2Degr=57.295779506;"""//PI=180°"""
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)
    
 
def sonner(chiffre):
    buzzer = Pin(2, Pin.OUT)
    if(chiffre == 0):
        buzzer.value(1)
        time.sleep(1)
        buzzer.value(0)
        time.sleep(0.2)
    else: 
        for i in range(chiffre):
            buzzer.value(1)
            time.sleep(0.2)
            buzzer.value(0)
            time.sleep(0.2)
        

def getAngleX():
    aX=round(imu.accel.x,2)/16384; """//16.384 LSB pour 1g d'après la doc"""

    aY=round(imu.accel.y,2)/16384;

    aZ=round(imu.accel.z,2)/16384;

    temp=math.sqrt(pow(aY,2) + pow(aZ,2));
    
    if(temp==0):
        return Rad2Degr * math.atan(aX/0.0000000000001);
    else:
        return Rad2Degr * math.atan(aX/temp);

def getAngleY():

    aX=round(imu.accel.x,2)/16384;"""//16.384 LSB pour 1g d'après la doc"""

    aY=round(imu.accel.y,2)/16384;

    aZ=round(imu.accel.z,2)/16384;

    temp= math.sqrt( pow(aX,2) + pow(aZ,2));
    if(temp==0):
        return Rad2Degr * math.atan(aY/0.0000000000001);
    else:
        return Rad2Degr * math.atan(aY/temp);



def getAngleZ():

    aX=round(imu.accel.x,2)/16384;"""//16.384 LSB pour 1g d'après la doc"""

    aY=round(imu.accel.y,2)/16384;

    aZ=round(imu.accel.z,2)/16384;
    
    temp= math.sqrt( pow(aX,2) + pow(aY,2));

    if(temp==0):
        return Rad2Degr * math.atan(aZ/0.0000000000001);
    else:
        return Rad2Degr * math.atan(aZ/temp);

        
def resultat():
    """
    if(getAngleX() > -1.3 and getAngleX()<1.0):
        return 5"""
    if(getAngleY()>85 and getAngleY() < 90):
        return 1
    if(getAngleZ() > -90 and getAngleZ() < -85):
        return 3
    if(getAngleX() > -90 and getAngleX() < -85):
        return 4
    if(getAngleZ() < 90 and getAngleZ() > 85):
        return 5
    if(getAngleY() < -85 and getAngleY() > -90):
        return 6
    if(getAngleX() < 90 and getAngleX() > 85):
        return 2
    print(0)
    return 0
    
def inter(premier, deuxieme, intervalle):
    max1 = premier + intervalle
    min1 = premier - intervalle
    return deuxieme > min1 and deuxieme < max1

def interAbsolue(nombre, target,intervalle):
    maxPos = target + intervalle
    minPos = target - intervalle
    maxNeg = -target + intervalle
    minNeg = -target - intervalle
    return (nombre > minPos and nombre < maxPos) or (nombre > minNeg and nombre < maxNeg)

def estStable():
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    imu = MPU6050(i2c)
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    time.sleep(5)
    ax2=round(imu.accel.x,2)
    ay2=round(imu.accel.y,2)
    az2=round(imu.accel.z,2)
    gx2=round(imu.gyro.x)
    gy2=round(imu.gyro.y)
    gz2=round(imu.gyro.z)
    return inter(ax, ax2, 20) and inter(ay, ay2, 20) and inter(az, az2, 20)


        

def afficherCapteur():
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    imu = MPU6050(i2c)
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    tem=round(imu.temperature,2)
    print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ",end="\n")
    sleep(0.2)
    
def estStable():
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    imu = MPU6050(i2c)
    monInter = 0.1
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    ax1 = interAbsolue(ax,1,monInter) and interAbsolue(ay,0,monInter) and interAbsolue(az,0,monInter)
    ay1 = interAbsolue(ax,0,monInter) and interAbsolue(ay,1,monInter) and interAbsolue(az,0,monInter)
    az1 = interAbsolue(ax,0,monInter) and interAbsolue(ay,0,monInter) and interAbsolue(az,1,monInter)
    return ax1 or ay1 or az1 
    

      
while True:
    aSonne = False
    while estStable():
        if(aSonne == False):
            sleep(2)
            while estStable() and aSonne == False:
                sonner(resultat())
                aSonne = True