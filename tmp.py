import smbus2
import time
import sys
sys.path.append("/root/")
from CocoPi import stm8s
iic_slaver=stm8s()
iic_slaver.clear()
del iic_slaver
from CocoPi import dcMotor
from CocoPi import extDcMotor
from CocoPi import multiFuncGpio
import serial
SERIAL = serial.Serial("/dev/ttyS1",115200)

def initVariables():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    clawAngleReset = 90
    clawAngleMax = 140
    clawAngleMin = 0
    clawAngle = clawAngleReset
    gripAngleOpen = 100
    gripAngleClose = 75
    movementSpeed = 100
    rcData = ""
    rcCommand = 100
    shooterAngleMax = 70
    shooterAngleMin = 0
    shooterAngle = shooterAngleMin
    shootingSpeed = 250
    shootingState = 0

M2 = dcMotor(2)
C = extDcMotor("C")
D = extDcMotor("D")
E = extDcMotor("E")
F = extDcMotor("F")

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
P2 = extServo(2)
P3 = extServo(3)
def initMotors():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    M2.dcMotorCtrl(1,0)
    C.speedControl(0)
    D.speedControl(0)
    E.speedControl(0)
    F.speedControl(0)
    P0.position(shooterAngle)
    P1.position(clawAngle)
    P2.position(gripAngleClose)
    P3.position(0)

def stopMoving():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(0)
    D.speedControl(0)
    E.speedControl(0)
    F.speedControl(0)

def moveClawDown():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    P1.position(clawAngleMin)

def moveShooterUp():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    if shooterAngle < shooterAngleMax:
        shooterAngle = shooterAngle + 10
    else:
        shooterAngle = shooterAngleMax
    P0.position(shooterAngle)
    time.sleep(50 / 1000)

def moveFront():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(-movementSpeed)
    D.speedControl(-movementSpeed)
    E.speedControl(-movementSpeed)
    F.speedControl(-movementSpeed)

def moveClawUp():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    P1.position(clawAngleMax)

def moveShooterDown():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    if shooterAngle > shooterAngleMin:
        shooterAngle = shooterAngle + -10
    else:
        shooterAngle = shooterAngleMin
    P0.position(shooterAngle)
    time.sleep(50 / 1000)

def moveBack():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(movementSpeed)
    D.speedControl(movementSpeed)
    E.speedControl(movementSpeed)
    F.speedControl(movementSpeed)

def openClaw():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    P2.position(gripAngleOpen)

def turnLeft():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(-60)
    D.speedControl(-60)
    E.speedControl(60)
    F.speedControl(60)

def activateShooter():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    M2.dcMotorCtrl(1,shootingSpeed)
    P3.position(180)
    time.sleep(1000 / 1000)
    P3.position(0)
    time.sleep(500 / 1000)
    M2.dcMotorCtrl(1,0)

def turnRight():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(60)
    D.speedControl(60)
    E.speedControl(-60)
    F.speedControl(-60)

def closeClaw():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    P2.position(gripAngleClose)

def shiftLeft():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(-movementSpeed)
    D.speedControl(movementSpeed)
    E.speedControl(-movementSpeed)
    F.speedControl(movementSpeed)

def releaseItem():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    P0.position(shooterAngleMax)
    time.sleep(50 / 1000)
    P1.position(clawAngleMax)
    time.sleep(50 / 1000)
    openClaw()

def shiftRight():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    C.speedControl(movementSpeed)
    D.speedControl(-movementSpeed)
    E.speedControl(movementSpeed)
    F.speedControl(-movementSpeed)

def resetShooterClaw():
    global clawAngle, clawAngleMax, clawAngleMin, clawAngleReset, command_timeout, current_action, gripAngleClose, gripAngleOpen, last_command_time, lines, movementSpeed, my_list, rcCommand, rcData, shooterAngle, shooterAngleMax, shooterAngleMin, shootingSpeed, shootingState, x
    P1.position(clawAngleReset)
    P0.position(shooterAngleMin)
    closeClaw()

def getcounterEndStart(newDate):
    global counter_start
    return newDate - counter_start

def _read_serial_data(read_data,split, index):
    if read_data != None:
        read_str = ""
        try:
            read_str = str(read_data.decode("utf-8")).split(split)[index]
        except:
            read_str = str(read_data).split(split)[index]
        return read_str



last_command_time = getcounterEndStart(time.perf_counter())
initVariables()
initMotors()
command_timeout = 0.2
clawAngle = 0
while True:
    SERIAL.flushInput()
    try:
        rcData = _read_serial_data(SERIAL.readline().decode("UTF-8","ignore").strip(),"|",1)
        print((str("Input: ") + str(rcData)))
        if (getcounterEndStart(time.perf_counter())) - last_command_time > command_timeout:
            stopMoving()
            current_action = 0
        if rcData != "NONE":
            last_command_time = getcounterEndStart(time.perf_counter())
            rcCommand = rcData.split(",")
            if rcCommand[0] == "L":
                if int(rcCommand[1]) > 100:
                    shiftRight()
                elif int(rcCommand[1]) < -100:
                    shiftLeft()
                elif int(rcCommand[2]) > 100:
                    moveBack()
                elif int(rcCommand[2]) < -100:
                    moveFront()
            elif rcCommand[0] == "R":
                if int(rcCommand[1]) > 100:
                    turnRight()
                elif int(rcCommand[1]) < -100:
                    turnLeft()
                elif int(rcCommand[2]) > 100:
                    stopMoving()
                elif int(rcCommand[2]) < -100:
                    stopMoving()
            else:
                if rcData == "NONE":
                    stopMoving()
                    shootingState = 0
                elif rcData == "UP":
                    moveClawUp()
                elif rcData == "DOWN":
                    moveClawDown()
                elif rcData == "LEFT":
                    openClaw()
                elif rcData == "RIGHT":
                    closeClaw()
                elif rcData == "L1":
                    releaseItem()
                elif rcData == "R1":
                    resetShooterClaw()
                elif rcData == "L2":
                    movementSpeed = 200
                elif rcData == "R2":
                    movementSpeed = 100
                elif rcData == "TRIANGLE":
                    moveShooterUp()
                elif rcData == "CROSS":
                    moveShooterDown()
                elif rcData == "SQUARE":
                    if shootingState == 0:
                        shootingSpeed = 250
                        shootingState = 1
                        activateShooter()
                elif rcData == "CIRCLE":
                    if shootingState == 0:
                        shootingSpeed = 170
                        shootingState = 1
                        activateShooter()
    except:
        rcData = "NONE"
        stopMoving()
