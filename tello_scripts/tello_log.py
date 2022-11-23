from djitellopy import Tello
import logging, time

tello = Tello(skipAddressCheck=True)
tello.LOGGER.setLevel(logging.INFO)
tello.connect()

print("Tello Battery Level = {}%".format(tello.get_battery()))

while True:
    time.sleep(0.5)