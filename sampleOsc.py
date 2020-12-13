import argparse
import time
import threading
import RPi.GPIO as GPIO
from pythonosc import udp_client

GPIO.setmode(GPIO.BCM)


class DistanceSensor:
    def __init__(self, gpio_trigger, gpio_echo):
        self.gpio_trigger = gpio_trigger
        self.gpio_echo = gpio_echo
        self.last_distance = 0
        self.stop_signal = False

        GPIO.setup(self.gpio_trigger, GPIO.OUT)
        GPIO.setup(self.gpio_echo, GPIO.IN)

    def measure(self):
        GPIO.output(self.gpio_trigger, True)

        time.sleep(0.00001)
        GPIO.output(self.gpio_trigger, False)

        start_time = time.time()
        stop_time = time.time()

        # save StartTime
        while GPIO.input(self.gpio_echo) == 0:
            start_time = time.time()

        # save time of arrival
        while GPIO.input(self.gpio_echo) == 1:
            stop_time = time.time()

        time_elapsed = stop_time - start_time

        self.last_distance = (time_elapsed * 34300) / 2

    def stop_sensing(self):
        self.stop_signal = True


sensor = DistanceSensor(gpio_trigger=18, gpio_echo=24)


def distance_sensor_thread_executor(distance_sensor: DistanceSensor):
    while not distance_sensor.stop_signal:
        distance_sensor.measure()
        time.sleep(0.02)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.1.6", help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=5005, help="The port the OSC server is listening on")
    args = parser.parse_args()

    client = udp_client.SimpleUDPClient(args.ip, args.port)

    thread = threading.Thread(target=distance_sensor_thread_executor, args=(sensor, ))
    thread.start()

    try:
        while True:
            client.send_message("/hello_world", sensor.last_distance)
            time.sleep(0.02)

    except KeyboardInterrupt:
        sensor.stop_sensing()
        print("Stopped by user")
        GPIO.cleanup()
