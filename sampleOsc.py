import argparse
import time
import concurrent.futures
import RPi.GPIO as GPIO
from pythonosc import udp_client

GPIO.setmode(GPIO.BCM)


class DistanceSensor:
    def __init__(self, gpio_trigger, gpio_echo):
        self.gpio_trigger = gpio_trigger
        self.gpio_echo = gpio_echo
        self.last_distance = 0

        self.setup_pins()

    def setup_pins(self):
        GPIO.setup(self.gpio_trigger, GPIO.OUT)
        GPIO.setup(self.gpio_echo, GPIO.IN)

    def measure(self):
        self.send_trigger_pulse()
        time_elapsed = self.measure_sound_roundtrip_time()
        self.calculate_distance(time_elapsed)

    def send_trigger_pulse(self):
        GPIO.output(self.gpio_trigger, True)
        time.sleep(0.00001)
        GPIO.output(self.gpio_trigger, False)

    def measure_sound_roundtrip_time(self):
        start_time = time.time()
        stop_time = time.time()
        # save StartTime
        while GPIO.input(self.gpio_echo) == 0:
            start_time = time.time()
        # save time of arrival
        while GPIO.input(self.gpio_echo) == 1:
            stop_time = time.time()
        time_elapsed = stop_time - start_time
        return time_elapsed

    def calculate_distance(self, time_elapsed):
        self.last_distance = (time_elapsed * 34300) / 2


class SensorDefinition:

    def __init__(self, sensor, address) -> None:
        self.sensor = sensor
        self.address = address


class ThreadSignaler:

    def __init__(self) -> None:
        self.allow_running = True

    def stop(self):
        self.allow_running = False


sensor_definitions = [
    SensorDefinition(sensor=(DistanceSensor(gpio_trigger=18, gpio_echo=24)), address='/sensor1'),
    SensorDefinition(sensor=(DistanceSensor(gpio_trigger=17, gpio_echo=27)), address='/sensor2'),
]

def distance_sensor_thread_runner(distance_sensor: DistanceSensor, signaler: ThreadSignaler):
    while signaler.allow_running:
        distance_sensor.measure()
        time.sleep(0.02)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.1.6", help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=5005, help="The port the OSC server is listening on")
    args = parser.parse_args()

    signaler = ThreadSignaler()

    client = udp_client.SimpleUDPClient(args.ip, args.port)

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(sensor_definitions)) as executor:
        for definition in sensor_definitions:
            executor.submit(distance_sensor_thread_runner, definition.sensor, signaler)

        try:
            while True:
                for definition in sensor_definitions:
                    client.send_message(definition.address, definition.sensor.last_distance)
                    time.sleep(0.02)

        except KeyboardInterrupt:
            signaler.stop()
            print("Stopped by user")
            GPIO.cleanup()

