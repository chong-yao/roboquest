from CocoPi import extDcMotor
import time
import smbus2
import sys
sys.path.append("/root/")
from CocoPi import multiFuncGpio
import base64
from maix import nn
from maix.nn import decoder
from maix import camera
from maix import image
import os
from maix import display

def initVariables():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    servoP0UnloadAngle = 115
    servoP0DownAngle = 26
    servoP1UpAngle = 60
    servoP1DownAngle = 0
    doWait = 0
    doTravel = "OPENED"
    currentPath = "SUPPLY"
    junctionCount = 0
    movementSpeed = 125
    correctionSpeed = 60
    vehicleLocation = ""
    containerFilled = "EMPTY"
    vehicleCommand = ""
    lineFollowThreshold = 250
    doDetect = 0

C = extDcMotor("C")
D = extDcMotor("D")
E = extDcMotor("E")
F = extDcMotor("F")
def stopMoving():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(0)
    D.speedControl(0)
    E.speedControl(0)
    F.speedControl(0)
    time.sleep(25 / 1000)

def moveFront():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(-movementSpeed)
    D.speedControl(-movementSpeed)
    E.speedControl(-movementSpeed)
    F.speedControl(-movementSpeed)

def moveBack():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(movementSpeed)
    D.speedControl(movementSpeed)
    E.speedControl(movementSpeed)
    F.speedControl(movementSpeed)

def turnLeft():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(-(movementSpeed - 35))
    D.speedControl(-(movementSpeed - 40))
    E.speedControl((movementSpeed - 40))
    F.speedControl((movementSpeed - 35))

def turnRight():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl((movementSpeed - 35))
    D.speedControl((movementSpeed - 40))
    E.speedControl(-(movementSpeed - 40))
    F.speedControl(-(movementSpeed - 35))

def correctLeft():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(-correctionSpeed)
    D.speedControl(-correctionSpeed)
    E.speedControl(correctionSpeed)
    F.speedControl(correctionSpeed)

def correctRight():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(correctionSpeed)
    D.speedControl(correctionSpeed)
    E.speedControl(-correctionSpeed)
    F.speedControl(-correctionSpeed)

def shiftLeft():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(-movementSpeed)
    D.speedControl(movementSpeed)
    E.speedControl(-movementSpeed)
    F.speedControl(movementSpeed)

def shiftRight():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    C.speedControl(movementSpeed)
    D.speedControl(-movementSpeed)
    E.speedControl(movementSpeed)
    F.speedControl(-movementSpeed)

class PCA9685(object):
    bus=smbus2.SMBus(2)
    def __init__(self,freq=400,min_us=460,max_us=2400,address=0x40,degrees=180):
        self.address=address
        self.period=1000000/freq
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.freq(freq)
        self.reset()
        #for i in range(0,16):
            #self.duty(i,0)
        print("Pca9685 init")

    def write(self, addr, val):
        for i in range(0, 2):
            try:
                self.bus.write_byte_data(self.address, addr, val)
                #time.sleep(0.001) # 1ms
                # print(addr, val) # debug
                return True
            except Exception:
                time.sleep(0.001)
                continue
        return False

    def read(self,addr):
        for i in range(0, 3):
            try:
                tmp = self.bus.read_byte_data(self.address, addr)
                #time.sleep(0.001) # 1ms
                # print(addr, tmp) # debug
                return tmp
            except Exception:
                time.sleep(0.01)
                continue
        return None

    def reset(self):
        self.write(0x00,0x00)        #初始化

    def freq(self,freq=None):
        if freq is None:
            return int(25000000.0/4096/(self.read(0xfe)-0.5))
        #设定频率freq,预分频prescale=int(25000000.0 / (4096.0 * freq) + 0.5)
        prescale=int(25000000.0/4096/freq+0.5)
        self.write(0x00,0x10)        #设定pca9685为睡眠模式
        self.write(0xfe,prescale)    #设定频率
        self.reset()
        time.sleep(0.01)
        self.write(0x00,0xa1)        #设定pca9685为活跃模式

    def pwm(self,index,on=None,off=None):            #on和off来调节PWM的占空比
        if not 0<= index <=15:
            raise ValueError("Pin ID out of range!")
        if on is None or off is None:
            data = self.bus.read_i2c_block_data(self.address,0x06+index*4,4)
            return data
        data= [0]*4
        data[0]=int(hex(on & 0xff),16)
        data[1]=int(hex((on >> 8) & 0xff),16)
        data[2]=int(hex(off & 0xff),16)
        data[3]=int(hex((off >> 8) & 0xff),16)
        # print(data)
        for i in range(0,4):
            self.write(0x06+i+index*4,data[i])

    def duty(self,index,value=None):
        if value == None:
            return self.pwm(index)
        elif not 0 <= value <=4095:
            raise ValueError("Out of range!")
        elif value==0:
            self.pwm(index,0,4096)
        elif value == 4095:
            self.pwm(index,4096,0)
        else:
            self.pwm(index,0,value)

    def _us2duty(self,value):
        return 4095*value/self.period

    def __del__(self):
        print("del pac9685")
        time.sleep(1)
        for i in range(0,16):
            self.duty(i,0)
        time.sleep(0.001)

class extServo(PCA9685):
    def __init__(self,servoId):
        PCA9685.__init__(self)
        self.servoId=servoId
        self.servoPin=[14,15,1,0]
        self.degrees=180
        pass

    def position(self,degrees=None):        #index:0,1,2,3
        if degrees == 180:
            self.max_duty = 3800
        span = self.max_duty - self.min_duty
        duty = self.min_duty + span * degrees / self.degrees
        duty = int(min(self.max_duty, max(self.min_duty, int(duty))))
        self.duty(self.servoPin[self.servoId], duty)

    def release(self):
        self.duty(self.servoPin[self.servoId],0)

    def __del__(self):
        self.duty(self.servoPin[self.servoId],0)

P0 = extServo(0)
P1 = extServo(1)
def initMotors():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    stopMoving()
    P0.position(servoP0DownAngle)
    P1.position(servoP1UpAngle)

def doRescue():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    if junctionCount == 1 or junctionCount == 2 or junctionCount == 4 or junctionCount == 5:
        moveFront()
        time.sleep(225 / 1000)
        turnRight()
        time.sleep(1850 / 1000)
        doTravel = "CLOSED"
        doDetect = 0
        if junctionCount == 5:
            doDetect = 1
    elif junctionCount == 3:
        doWait = 1
        moveFront()
        time.sleep(300 / 1000)
        stopMoving()
        time.sleep(1000 / 1000)
        doTravel = "CLOSED"
        doDetect = 1

def doUnload():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    moveFront()
    time.sleep(250 / 1000)
    shiftLeft()
    time.sleep(225 / 1000)
    stopMoving()
    time.sleep(500 / 1000)
    P0.position(servoP0UnloadAngle)
    time.sleep(2000 / 1000)
    P0.position(servoP0DownAngle)
    time.sleep(500 / 1000)
    shiftRight()
    time.sleep(225 / 1000)
    junctionCount = 0
    containerFilled = "EMPTY"
    vehicleCommand = ""

def doSupply():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    if junctionCount <= 1:
        moveFront()
        time.sleep(225 / 1000)
        turnLeft()
        time.sleep(1850 / 1000)
        doTravel = "CLOSED"
        doDetect = 0
    elif junctionCount == 5:
        moveFront()
        time.sleep(225 / 1000)
        turnRight()
        time.sleep(1850 / 1000)
        doTravel = "CLOSED"
        doDetect = 1
    elif junctionCount == 2 or junctionCount == 4:
        moveFront()
        time.sleep(300 / 1000)
    elif junctionCount == 3:
        stopMoving()
        P1.position(servoP1DownAngle)
        time.sleep(300 / 1000)
        moveBack()
        time.sleep(1000 / 1000)
        for count in range(2):
            P1.position(servoP1UpAngle)
            moveFront()
            time.sleep(1000 / 1000)
            P1.position(servoP1DownAngle)
            stopMoving()
            time.sleep(100 / 1000)
            moveBack()
            time.sleep(1000 / 1000)
        moveBack()
        time.sleep(400 / 1000)
        shiftRight()
        time.sleep(150 / 1000)
        turnRight()
        time.sleep(3700 / 1000)
        P1.position(servoP1UpAngle)
        doTravel = "CLOSED"
        containerFilled = "FILLED"

def doObstacle():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    if junctionCount == 1 or junctionCount == 2:
        moveFront()
        time.sleep(225 / 1000)
        turnRight()
        time.sleep(1850 / 1000)
        doTravel = "CLOSED"
        doDetect = 0
    elif junctionCount == 3:
        doWait = 1
        moveFront()
        time.sleep(300 / 1000)
        stopMoving()
        time.sleep(1000 / 1000)
        doTravel = "CLOSED"
        doDetect = 1
    elif junctionCount == 4:
        moveFront()
        time.sleep(225 / 1000)
        turnLeft()
        time.sleep(1850 / 1000)
        doTravel = "CLOSED"
        doDetect = 1

def lcdRotation(inputImg):
    global SETVFLIP,SETHMIRROT,cameraSize,ScreenOrientation
    imageRotationBuffer = inputImg.crop(0, 0, 240, 320)
    if ScreenOrientation:
        imgRotationAim = image.new(size = (240, 320))
        rotationAngle = 180
    else:
        imgRotationAim = image.new(size = (320, 240))
        rotationAngle = 90
    GETROTATION = imageRotationBuffer.rotate(+rotationAngle, adjust=1)
    GETROTATION = imgRotationAim.draw_image(GETROTATION,0,0,alpha=1)
    if SETVFLIP and not SETHMIRROT:
        GETROTATIONs = GETROTATION.flip(0)
    if SETHMIRROT and not SETVFLIP:
        GETROTATIONs = GETROTATION.flip(1)
    if SETVFLIP and SETHMIRROT:
        GETROTATION1 = GETROTATION.flip(0)
        GETROTATION = GETROTATION1.flip(1)
    return GETROTATION

def lcdRotationNew(inputImg):
    global SETVFLIP,SETHMIRROT,cameraSize,ScreenOrientation
    imageRotationBuffer = inputImg.crop(0, 0, 320, 240)
    if ScreenOrientation:
        imgRotationAim = image.new(size = (240, 320))
        rotationAngle = 90
        GETROTATION = imageRotationBuffer.rotate(+rotationAngle, adjust=1)
    else:
        imgRotationAim = image.new(size = (320, 240))
        GETROTATION = imageRotationBuffer

    GETROTATION = imgRotationAim.draw_image(GETROTATION,0,0,alpha=1)
    if SETVFLIP and not SETHMIRROT:
        GETROTATIONs = GETROTATION.flip(0)
    elif SETHMIRROT and not SETVFLIP:
        GETROTATIONs = GETROTATION.flip(1)
    elif SETVFLIP and SETHMIRROT:
        GETROTATION1 = GETROTATION.flip(0)
        GETROTATION = GETROTATION1.flip(1)
    return GETROTATION

def getLcdRotation(cameraCapture):
    global cameraSize
    if cameraSize:
        return lcdRotationNew(cameraCapture)
    else:
        return lcdRotation(cameraCapture)

def v831_display_show_imageDetection(displayShow):
    global _imageDetection_y,_imageDetection_x,ScreenOrientation,cameraSize
    CANVASSHOWIMGAGE = ""
    if ScreenOrientation:
        displayShowCanvas = image.new(size = (240, 320))
        displayShowCanvas.draw_rectangle(0,0,240,320, color=(0,0,0), thickness=-1)
        displayShowCanvas.draw_image(displayShow,_imageDetection_x,_imageDetection_y,alpha=1)
        displayShowVER = displayShowCanvas.crop(0,0,240,320)
        displayShowVER = displayShowVER.rotate(-90, adjust=1)
        display.show(displayShowVER)
    else:
        displayShowCanvas = image.new(size = (320, 240))
        displayShowCanvas.draw_rectangle(0,0,320,240, color=(0,0,0), thickness=-1)
        displayShowCanvas.draw_image(displayShow,_imageDetection_x,_imageDetection_y,alpha=1)
        display.show(displayShowCanvas)


def doDetection():
    global doDetect, lineFollowThreshold, vehicleCommand, containerFilled, vehicleLocation, correctionSpeed, movementSpeed, junctionCount, currentPath, doTravel, doWait, servoP1DownAngle, servoP1UpAngle, servoP0DownAngle, servoP0UnloadAngle, imageDetection, wait, i, count, _imageDetection_x, _imageDetection_y, SETVFLIP, SETHMIRROT, ScreenOrientation, _PWN_gpio_1, _PWN_gpio_2
    imageDetection = getLcdRotation(camera.capture())
    imageDetection = imageDetection.crop(48, 8,224, 224)
    out = Yolo.model.forward(imageDetection, quantize=True, layout="hwc")
    boxes, probs = Yolo.decoder.run(out, nms=0.3, threshold=0.3, img_size=(224, 224))
    if len(boxes):
        for boxesi, box in enumerate(boxes):
            boxes[boxesi].append(probs[boxesi])
    if (len(boxes)) and doDetect == 1:
        for i in (boxes):
            imageDetection.draw_string((i[0]),(i[1]), (str(Yolo.labels[i[4][0]])), scale = 1, color = (255,0,0) , thickness = 1)
            imageDetection.draw_rectangle((i[0]),(i[1]),(i[0]+i[2]), (i[1]+i[3]), color=(255,0,0), thickness=1)
            if (Yolo.labels[i[4][0]]) == "Supply_Station" and (i[0]) < 200:
                vehicleLocation = "SUPPLY_STATION"
            elif (Yolo.labels[i[4][0]]) == "Rescue_Station" and (i[1]) < 200:
                vehicleLocation = "RESCUE_STATION"
            elif (Yolo.labels[i[4][0]]) == "Clearing_Zone" and (i[1]) < 200:
                vehicleLocation = "CLEARING_ZONE"
            elif (Yolo.labels[i[4][0]]) == "D":
                vehicleLocation = "RETURN"
                doDetect = 0
            elif (Yolo.labels[i[4][0]]) == "A":
                vehicleCommand = "GO_RESCUE"
            elif (Yolo.labels[i[4][0]]) == "B":
                vehicleCommand = "GO_COLLECT"
            elif (Yolo.labels[i[4][0]]) == "C":
                vehicleCommand = "GO_CLEAR"
    else:
        vehicleLocation = ""
    if vehicleLocation != "":
        if vehicleLocation == "SUPPLY_STATION" and currentPath == "SUPPLY" and containerFilled == "FILLED":
            if vehicleCommand == "GO_RESCUE":
                doUnload()
                currentPath = "RESCUE"
            elif vehicleCommand == "GO_CLEAR":
                doUnload()
                currentPath = "OBSTACLE"
            else:
                doUnload()
                turnLeft()
                time.sleep(3700 / 1000)
                doTravel = "CLOSED"
                currentPath = "SUPPLY"
        elif vehicleLocation == "RESCUE_STATION" and currentPath == "RESCUE" and containerFilled == "FILLED":
            if vehicleCommand == "GO_CLEAR":
                doUnload()
                currentPath = "OBSTACLE"
            elif vehicleCommand == "GO_COLLECT":
                doUnload()
                turnLeft()
                time.sleep(3700 / 1000)
                doTravel = "CLOSED"
                currentPath = "SUPPLY"
            else:
                doUnload()
                currentPath = "RESCUE"
        elif vehicleLocation == "CLEARING_ZONE" and currentPath == "OBSTACLE" and containerFilled == "FILLED":
            doUnload()
            currentPath = "SUPPLY"
            junctionCount = -1
        elif vehicleLocation == "RETURN":
            wait = 0
            if currentPath == "RESCUE":
                containerFilled = "FILLED"
                moveFront()
            elif currentPath == "OBSTACLE":
                containerFilled = "FILLED"
                doTravel = "CLOSED"
                turnRight()
                time.sleep(3700 / 1000)
    _imageDetection_x, _imageDetection_y = 48,8
    v831_display_show_imageDetection(imageDetection)

cameraSize = True
def CAMERATYPE():
    global cameraSize
    try:
        if os.path.exists("/etc/cameraSize.cfg"):
            cameraSize = True
        else:
            cameraSize = False
    except:
        cameraSize = False
image.load_freetype("/root/preset/fonts/simhei.ttf")
CAMERATYPE()

_imageDetection_x = 0
_imageDetection_y = 0
SETVFLIP = False
SETHMIRROT = False
ScreenOrientation = False
_PWN_gpio_1 = multiFuncGpio(0,4)
_PWN_gpio_2 = multiFuncGpio(1,4)



initVariables()
initMotors()
class Yolo:
    labels = ["Supply_Station", "Rescue_Station", "Clearing_Zone", "A", "B", "C", "D"]
    anchors = [1.19, 1.98, 2.79, 4.59, 4.53, 8.92, 8.06, 5.29, 10.32, 10.65]
    m = {
        "param": "/root/preset/model/cocopi_FloodingMission.param",
        "bin": "/root/preset/model/cocopi_FloodingMission.bin"
    }
    options = {
        "model_type":  "awnn",
        "inputs": {
            "input0": (224, 224, 3)
        },
        "outputs": {
            "output0": (7, 7, (1+4+len(labels))*5)
        },
        "mean": [127.5, 127.5, 127.5],
        "norm": [0.0078125, 0.0078125, 0.0078125],
    }
    def __init__(self):
        from maix import nn
        from maix.nn import decoder
        self.model = nn.load(self.m, opt=self.options)
        self.decoder = decoder.Yolo2(len(self.labels), self.anchors, net_in_size=(224, 224), net_out_size=(7, 7))
    def __del__(self):
        del self.model
        del self.decoder
Yolo = Yolo()

if cameraSize==True:
    camera.camera.config(size=(320,240))
else:
    camera.camera.config(size=(240,320))
while True:
    doDetection()
    if (_PWN_gpio_1.analogRead()) < lineFollowThreshold and (_PWN_gpio_2.analogRead()) < lineFollowThreshold:
        junctionCount = junctionCount + 1
        if currentPath == "SUPPLY":
            doSupply()
        elif currentPath == "RESCUE":
            doRescue()
        elif currentPath == "OBSTACLE":
            doObstacle()
    elif ((_PWN_gpio_1.analogRead()) < lineFollowThreshold or (_PWN_gpio_2.analogRead()) < lineFollowThreshold) and doWait == 0:
        doTravel = "OPENED"
    if doTravel == "OPENED":
        if (_PWN_gpio_1.analogRead()) > lineFollowThreshold and (_PWN_gpio_2.analogRead()) < lineFollowThreshold:
            correctRight()
        elif (_PWN_gpio_1.analogRead()) < lineFollowThreshold and (_PWN_gpio_2.analogRead()) > lineFollowThreshold:
            correctLeft()
        else:
            moveFront()
