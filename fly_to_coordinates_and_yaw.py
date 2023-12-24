import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import math
import argparse


def connect_uav():
    parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    return connect(connection_string, wait_ready=True)

# Connecting to a vehicle
vehicle = connect_uav()

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def do_yaw(heading):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees) ralatively to vehicle
    """
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,          #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        1,          # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def move_to_point(point):
    """
    Moving to a point in  the AltHold mode
    """

    print("Moving to location lat: %s and lon: %s".format(point.lat, point.lon))
    vehicle.simple_goto(point)

    # Wait until the vehicle reaches the target point
    while True:
        remaining_distance = get_distance_metres(vehicle.location.global_relative_frame, point)
        print(f"Distance to target: {remaining_distance} meters")

        if remaining_distance < 1:
            print("Reached target!")
            break

        time.sleep(1)

arm_and_takeoff(100)

move_to_point(LocationGlobalRelative(50.443326, 30.448078, 100))

print("Yaw 350 digrees relative to the UAV")
do_yaw(350)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
