import RPi.GPIO as GPIO
import Adafruit_DHT
import lcddriver
import time
from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)

GPIO.setmode(GPIO.BCM)

# For the Micro Servo
GPIO.setup(21, GPIO.OUT)
pwm_pin = GPIO.PWM(21, 50)
#Sets the Micro Servo to 90
pwm_pin.start(7.5)

# For the Temperature and Humidity Sensor
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 26

#set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# For the 16x4 LCD Display
display = lcddriver.lcd()
# display = I2C_LCD_driver.lcd()

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

if __name__ == "__main__":  
    try:
        while True:
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
            print("Writing the Temperature and Humidity to the display")
            display.lcd_display_string("Temp: {0:0.1f} *C".format(temperature), 1)
            display.lcd_display_string("Humidity: {0:0.1f}%".format(humidity), 2)
            fahrenheit_angle = temperature * 9/5 + 32
            print("Fahrenheit angle: {0:0.1f}".format(fahrenheit_angle))
            pwm_pin.ChangeDutyCycle(fahrenheit_angle / 18 + 2)
            time.sleep(5)
            print("Resetting the Angle back to 90 Degrees")
            pwm_pin.ChangeDutyCycle(7.5)
            time.sleep(1)
            display.lcd_clear() 
            dist = distance()
            print ("Measured Distance = %.1f cm" % dist)
	    accelerometer_data = sensor.get_accel_data()
	    print(accelerometer_data)
            display.lcd_display_string("Distance: %.1f cm" % dist, 1)
	    display.lcd_display_string("Vezi Terminalul", 2)
            time.sleep(5)
            display.lcd_clear() 
            time.sleep(1) 

    except KeyboardInterrupt: 
        print("Cleaning up!")
    finally:
        display.lcd_clear()
        pwm_pin.stop() 
        GPIO.cleanup()



