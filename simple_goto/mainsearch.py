#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
6/8/17:
mainsearch.py: this version demonstrate two vehicles both
executing goto commands.

© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit.mavlink import MAVConnection


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--gcs',
                   help="mavlink is relayed to this address for gcs to monitor ")
parser.add_argument('--beaconxy',
                   help="relative location of beacon from home ")
parser.add_argument('--targethome',
                   help="relative location of beacon from home ")

args = parser.parse_args()

connection_string = args.connect
connection_string2 = args.connect
gcs = args.gcs
beaconxy=args.beaconxy
beaconx=int(beaconxy.split(',')[0])
beacony=int(beaconxy.split(',')[1])
sitl = None
print ('beaconxy is %d %d' %(beaconx,beacony))
#exit (0)

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
out = MAVConnection(gcs, source_system=255)
#out2 = MAVConnection(sys.argv[3], source_system=253)
vehicle._handler.pipe(out)
out.start()

# Start SITL if no connection string specified
# create a vehicle to represent the beacon target
if not connection_string2:
    import dronekit_sitl
# start_default2(lat=None, lon=None):
    sitl2 = dronekit_sitl.start_default2(-35.361354,149.167218)
    connection_string2 = sitl2.connection_string2()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string2)
vehicle2 = connect(connection_string2, wait_ready=True)
out2 = MAVConnection(gcs, source_system=254)
#out2 = MAVConnection(sys.argv[3], source_system=253)
vehicle2._handler.pipe(out2)
out2.start()
#vehicle2._location  = LocationGlobalRelative(-35.361354, 149.167218, 20)
print(vehicle._location._lat)
print(vehicle._location._lon)
print(vehicle2._location._lat)
print(vehicle2._location._lon)
print('v2 location')
def arm_and_takeoff(vehicle, aTargetAltitude):
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

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(vehicle, 10)
arm_and_takeoff(vehicle2, 10)

print(vehicle._location._lat)
print(vehicle._location._lon)
print(vehicle2._location._lat)
print(vehicle2._location._lon)
print('v2 location')
print("Set default/target airspeed to 3")
vehicle.airspeed = 3
vehicle2.airspeed = 5 

time.sleep(10)
print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
point11 = LocationGlobalRelative(-35.361354, 149.166218, 20)
vehicle.simple_goto(point1)
vehicle2.simple_goto(point11)

# sleep so we can see the change in map
time.sleep(30)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(point2, groundspeed=10)
vehicle2.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
vehicle2.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
vehicle2.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
    sitl2.stop()
