import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import argparse
import enum

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

def channel_override(overrides):
    vehicle.channels.overrides = { **vehicle.channels.overrides, **overrides }
    print(vehicle.channels.overrides)

def arm_and_takeoff(altitude):
    """
    Arms vehicle and fly to altitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    
    print("Cleaning all channel overrides")
    channel_override({ '2': 1500, '3': 1500, '4': 1500 })

    while True:
        channel_override({ '3': 1800 })
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    channel_override({ '3': 1500 })

class RangeCheck(enum.Enum):
    IN_RANGE = 1
    NEED_LESS = 2
    NEED_MORE = 3

def is_heading_in_range(from_degrees, to_degrees, current_degrees):
    if (from_degrees > to_degrees): # the case, when from is less than 0 degress (which becomes from + 360)
        if current_degrees > from_degrees or current_degrees < to_degrees:
            return RangeCheck.IN_RANGE
        else: 
            delta_to_mediana = (from_degrees - to_degrees) / 2
            return RangeCheck.NEED_MORE if from_degrees - delta_to_mediana < current_degrees else RangeCheck.NEED_LESS
    else:
        if current_degrees < from_degrees:
            return RangeCheck.NEED_MORE
        elif current_degrees > to_degrees:
            return RangeCheck.NEED_LESS
        else:
            return RangeCheck.IN_RANGE

def do_yaw(heading, allowed_deviation_in_degrees = 10, forse_clockwise = False):
    print(f"Do yaw up to {heading} degrees...")
    # Wait until the vehicle reaches the target point
    if (allowed_deviation_in_degrees >= 10):
        speed_delta = 50
    else:
        speed_delta = 40

    while True:
        print("Curr heading in degrees: ", vehicle.heading)
        # Break and return from function just below target altitude.
        from_range = bearing_plus_delta_and_normalize(heading, -allowed_deviation_in_degrees)
        to_range =  bearing_plus_delta_and_normalize(heading, allowed_deviation_in_degrees)
        range_check = is_heading_in_range(from_range, to_range, vehicle.heading)
        if range_check == RangeCheck.IN_RANGE:
            print("Reached target heading")
            channel_override({ '4': 1500 })
            break
        elif range_check == RangeCheck.NEED_MORE or forse_clockwise:
            print(f"Heading right")
            channel_override({ '4': 1500 + speed_delta })
        elif range_check == RangeCheck.NEED_LESS:
            print(f"Heading left")
            channel_override({ '4': 1500 - speed_delta })
            
        time.sleep(0.3)

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

def distance_bearing(homeLatitude, homeLongitude, destinationLatitude, destinationLongitude):

    """
    Simple function which returns the distance and bearing between two geographic location

    Inputs:
        1.  homeLatitude            -   Latitude of home location
        2.  homeLongitude           -   Longitude of home location
        3.  destinationLatitude     -   Latitude of Destination
        4.  destinationLongitude    -   Longitude of Destination

    Outputs:
        1. [Distance, Bearing]      -   Distance (in metres) and Bearing angle (in degrees)
                                        between home and destination

    Source:
        https://github.com/TechnicalVillager/distance-bearing-calculation
    """

    rlat1   =   homeLatitude * (math.pi/180) 
    rlat2   =   destinationLatitude * (math.pi/180) 
    rlon1   =   homeLongitude * (math.pi/180) 
    rlon2   =   destinationLongitude * (math.pi/180)

    # Formula for bearing
    y = math.sin(rlon2 - rlon1) * math.cos(rlat2)
    x = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(rlon2 - rlon1)
    
    # Bearing in radians
    bearing = math.atan2(y, x)
    bearingDegrees = bearing * (180/math.pi)
    bearingDegreesNormalisedToPositive = bearing_plus_delta_and_normalize(bearingDegrees)
    return bearingDegreesNormalisedToPositive

def bearing_plus_delta_and_normalize(bearing, delta=0):
    """
    Guarantee that bearing is in the 0 - 360 range
    """
    return (bearing + delta + 360) % 360

def move_to_point(point):
    """
    Moving to a point in  the AltHold mode
    """

    print("Moving to location...")

    # Wait until the vehicle reaches the target point
    while True:
        bearing_degrees = distance_bearing(
            vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon,
            point.lat, point.lon
        )
        remaining_distance = get_distance_metres(vehicle.location.global_relative_frame, point)
        
        print(f"Distance to target: {remaining_distance} meters")
        print(f"Bearing degress   : {bearing_degrees}")

        if remaining_distance < 1:
            print("\n******** Reached target! *****\n")
            channel_override({ '2': 1510 })
            time.sleep(0.1)
            channel_override({ '2': 1500 })
            break

        # constantly track bearing. If it riches the allowed deviation, we should do way to required bearing delta
        allowed_yaw_deviation_in_degrees = 10
        yaw_deviation_corrective_treshold = allowed_yaw_deviation_in_degrees + 5 # adding 5 here just to have some space and not constantly fix bearing
        if abs(bearing_plus_delta_and_normalize(bearing_degrees, -vehicle.heading)) > yaw_deviation_corrective_treshold: 
            do_yaw(bearing_degrees, allowed_yaw_deviation_in_degrees)

        if remaining_distance < 100:
            pitch = round(1425 + 25 - (25 * remaining_distance / 100))  # The closer the target, the lower the UAV's speed should be 
            channel_override({ '2': pitch })
        else:
            channel_override({ '2': 1000 })

        time.sleep(1)

arm_and_takeoff(100)
move_to_point(LocationGlobalRelative(50.443326, 30.448078, 100))

# Yaw
print("Yaw 350 digrees relative to the UAV")
print(f"Current bearing: {vehicle.heading}")
target_bearing = bearing_plus_delta_and_normalize(vehicle.heading, 350)
print(f"Target bearing: {target_bearing}")
do_yaw(target_bearing, 1, forse_clockwise=True)

# Close vehicle object before exiting script
print("\n\nAll tasks were completed")
print("Closing vehicle object....")
vehicle.close()
