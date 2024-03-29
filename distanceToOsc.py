import argparse
import time
import concurrent.futures
import RPi.GPIO as GPIO
from pythonosc import udp_client, osc_bundle_builder, osc_message_builder

import board
import busio

import adafruit_vl53l0x

GPIO.setmode(GPIO.BCM)

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)

class DistanceSensorUltraSonic:
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

class DistanceSensorLight:
    def __init__(self) -> None:
        self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)

        self.last_distance = 0

    def measure(self):
        self.last_distance = self.vl53.distance


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
    SensorDefinition(sensor=(DistanceSensorUltraSonic(gpio_trigger=18, gpio_echo=24)), address='/sensor1'),
    SensorDefinition(sensor=(DistanceSensorUltraSonic(gpio_trigger=17, gpio_echo=27)), address='/sensor2'),
    SensorDefinition(sensor=(DistanceSensorUltraSonic(gpio_trigger=12, gpio_echo=6)), address='/sensor3'),
    SensorDefinition(sensor=(DistanceSensorLight()), address='/lightSensor1'),
]

def distance_sensor_thread_runner(distance_sensor: DistanceSensorUltraSonic, signaler: ThreadSignaler):
    while signaler.allow_running:
        distance_sensor.measure()
        time.sleep(0.05)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="255.255.255.250", help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=5005, help="The port the OSC server is listening on")
    args = parser.parse_args()

    signaler = ThreadSignaler()

    client = udp_client.UDPClient(args.ip, args.port)

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(sensor_definitions)) as executor:
        for definition in sensor_definitions:
            executor.submit(distance_sensor_thread_runner, definition.sensor, signaler)

        try:
            while True:
                bundle_builder = osc_bundle_builder.OscBundleBuilder(osc_bundle_builder.IMMEDIATELY)
                for definition in sensor_definitions:
                    message_builder = osc_message_builder.OscMessageBuilder(definition.address)
                    message_builder.add_arg(definition.sensor.last_distance)
                    message = message_builder.build()
                    bundle_builder.add_content(content=message)
                client.send(bundle_builder.build())
                time.sleep(0.05)

        except KeyboardInterrupt:
            signaler.stop()
            print("Stopped by user")
            GPIO.cleanup()

