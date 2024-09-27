import RPi.GPIO as GPIO
import time

# GPIO Pin setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)  # GPIO23 for forward
GPIO.setup(24, GPIO.OUT)  # GPIO24 for reverse

# Set up PWM on the GPIO pins
pwm_forward = GPIO.PWM(23, 100)  # PWM on GPIO23 with 100Hz frequency
pwm_reverse = GPIO.PWM(24, 100)  # PWM on GPIO24 with 100Hz frequency

# Start PWM with 0% duty cycle (motor off)
pwm_forward.start(0)
pwm_reverse.start(0)

try:
    # Turn the motor on (forward) by increasing PWM duty cycle
    print("Turning motor forward...")
    pwm_forward.ChangeDutyCycle(75)  # 75% duty cycle for forward movement
    time.sleep(5)  # Run for 5 seconds

    # Stop the motor by setting PWM duty cycle to 0
    print("Stopping motor...")
    pwm_forward.ChangeDutyCycle(0)
    time.sleep(2)

    # Turn the motor on (reverse)
    print("Turning motor reverse...")
    pwm_reverse.ChangeDutyCycle(75)  # 75% duty cycle for reverse movement
    time.sleep(5)

    # Stop the motor again
    print("Stopping motor...")
    pwm_reverse.ChangeDutyCycle(0)

finally:
    # Cleanup GPIO settings
    pwm_forward.stop()
    pwm_reverse.stop()
    GPIO.cleanup()