#Libraries
import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)


### 
#set GPIO Pins
##set motor driver pins
enL=12
inL1=5
inL2=6

enR=13
inR1=4
inR2=26


##set HC04 pins
GPIO_TRIGGER_left = 23
GPIO_ECHO_left = 24
#yanyana pin
GPIO_TRIGGER_right = 27
GPIO_ECHO_right = 22
 #yan yana pin


#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER_left, GPIO.OUT)
GPIO.setup(GPIO_ECHO_left, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_right, GPIO.OUT)
GPIO.setup(GPIO_ECHO_right, GPIO.IN)
GPIO.setup(inL1,GPIO.OUT)
GPIO.setup(inL2,GPIO.OUT)
GPIO.setup(enL,GPIO.OUT)
GPIO.setup(inR1,GPIO.OUT)
GPIO.setup(inR2,GPIO.OUT)
GPIO.setup(enR,GPIO.OUT)

GPIO.output(inL1,GPIO.LOW)
GPIO.output(inL2,GPIO.LOW)
GPIO.output(inR1,GPIO.LOW)
GPIO.output(inR2,GPIO.LOW)

speedL=GPIO.PWM(enL,10000)
speedR=GPIO.PWM(enR,10000)

def turn_left():
    GPIO.output(inL1,GPIO.LOW)
    GPIO.output(inL2,GPIO.HIGH)
    GPIO.output(inR1,GPIO.HIGH)
    GPIO.output(inR2,GPIO.LOW)
def turn_right():
    GPIO.output(inL1,GPIO.HIGH)
    GPIO.output(inL2,GPIO.LOW)
    GPIO.output(inR1,GPIO.LOW)
    GPIO.output(inR2,GPIO.HIGH)
def go_straight(speed):
    GPIO.output(inL1,GPIO.HIGH)
    GPIO.output(inL2,GPIO.LOW)
    GPIO.output(inR1,GPIO.HIGH)
    GPIO.output(inR2,GPIO.LOW)
    speedL.ChangeDutyCycle(speed)
    speedR.ChangeDutyCycle(speed)
def go_back():
    GPIO.output(inL1,GPIO.LOW)
    GPIO.output(inL2,GPIO.HIGH)
    GPIO.output(inR1,GPIO.LOW)
    GPIO.output(inR2,GPIO.HIGH)
def turn_degrees(degree):
    if(abs(degree)<10):
        pass
    else:
        if((degree)<0):
            turn_left()
            time.sleep(1)
        else:
            turn_right()
            time.sleep(1)

def follow(distance,err_prev, err_total):
    Kp=1
    KD=0
    KI=0
    required_dist=50
    err=distance-required_dist
    err_total+=err
    speed=err*Kp+err_prev*KD+err_total*KI

    ##TODO map speed to 0 100
    if(abs(err)<required_dist):
        speed=0
    go_straight(speed)
    time.sleep(2)

    err_prev=err
    return err,err_prev,err_total
def distance_left():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER_left, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_left, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO_left) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO_left) == 1:
        StopTime = time.time()
 

    TimeElapsed = StopTime - StartTime

    distance = (TimeElapsed * 34300) / 2
 
    return distance_left
def distance_right():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER_right, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_right, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO_right) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO_right) == 1:
        StopTime = time.time()
 

    TimeElapsed = StopTime - StartTime

    distance = (TimeElapsed * 34300) / 2
 
    return distance_right 



if __name__ == '__main__':
    try:
        while True:
            degree=0
            distance=0
            turn_degrees(degree)
            follow(distance)
            time.sleep(1)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()