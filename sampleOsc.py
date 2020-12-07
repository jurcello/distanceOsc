import argparse
import time
import RPi.GPIO as GPIO
from pythonosc import udp_client

GPIO.setmode(GPIO.BCM)

GPIO_TRIGGER = 18
GPIO_ECHO = 24

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


def distance():
    GPIO.output(GPIO_TRIGGER, True)

    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = (time_elapsed * 34300) / 2

    return distance


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.1.6", help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=5005, help="The port the OSC server is listening on")
    args = parser.parse_args()

    client = udp_client.SimpleUDPClient(args.ip, args.port)

    try:
        while True:
            dist = distance()
            client.send_message("/hello_world", dist)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopped by user")
        GPIO.cleanup()