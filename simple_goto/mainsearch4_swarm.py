#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
6/9/17:
mainsearch2.py: this version demonstrate two vehicless 
v#1 going to V#2 location, the V#2 location will be
replace by an rssi function later so a rssi search
can be simulated.

© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit.mavlink import MAVConnection
from scipy.spatial.distance import pdist
import random
import math 
import cmd_rssisim
from cmd_rssisim import RssiRecord
# Set up option parsing to get connection string
import argparse
import utm

#range0 to 90 meters, rssi 65 to 0
#linea equation approx: slope= -65/90
def rssi(srcgps, destgps):
	srcxy = gps_to_local(srcgps[0], srcgps[1])
	print("src x,y: %f %f" %(srcxy[0],srcxy[1]))
	destxy = gps_to_local(destgps[0], destgps[1])
	print("dest x,y: %f %f" %(destxy[0],destxy[1]))
	rssiv = rssi_internal(srcxy, destxy)
 	return rssiv, srcxy

def rssi_internal(srcxy, destxy):
	slope = -65.0/90
	dist=pdist([srcxy,destxy])
	dist = dist/3
	print ("dist scaled by 3: %f" %dist)
	dist = min(dist, 500)
	print ("dist : %f" %dist)
	rssi = 65 + dist*slope
	print ("rssi : %f" %rssi)
	#rssi = dist 
	rssi = rssi + 2*random.random()
	print ("rssi with noise : %f" %rssi)
 	return rssi

gps_ref_lat=-35.371
gps_ref_lon=149.177
utm_ref = utm.from_latlon(gps_ref_lat, gps_ref_lon)
cmd_rssisim.loadrssi_map_f()
print (cmd_rssisim.rssilist_map[0])

def rssi_xygps(srcxy, destgps):
	destxy = gps_to_local(destgps[0], destgps[1])
	print("dest x,y: %f %f" %(destxy[0],destxy[1]))
	rssiv = rssi_internal(srcxy, destxy)
 	return rssiv, srcxy

def local_to_gps(x, y):
	lat, lon = utm.to_latlon(utm_ref[0]+x, utm_ref[1]+y, utm_ref[2], utm_ref[3])
 	return [lat,lon]

def gps_to_local(lat, lon):
#  the haversine is not consistent with the conversion funcs in utm package 
#  mixing them cause offset and error in the local/latlon calculated, so we
#  use utm pacage for all now.

        (x_utm,y_utm, z_utm, zone_utm)= utm.from_latlon(lat,lon)

        (refx_utm,refy_utm, refz_utm, refzone_utm)= utm.from_latlon(gps_ref_lat,gps_ref_lon)
        x=x_utm-refx_utm
        y=y_utm-refy_utm

#        lat3 = gps_ref_lat
#        lon3 = lon
#        x= math.copysign(1, lon-gps_ref_lon) * haversine(gps_ref_lat, gps_ref_lon, lat3,lon3)
#        y= math.copysign(1, lat-gps_ref_lat) *haversine(lat3, lon3, lat, lon)
        return [x,y]


def haversine(lat1, lon1, lat2, lon2):
#calculate dist between two points based on gps lat and lon_lon
        # Convert all decimal degrees to radians
        #test case: 37.2398256  -77.4185491
        #37.239869      -77.4183346 , dist 19.6 meters
        lat1_rad = lat1 * math.pi /180
        lat2_rad = lat2 * math.pi /180
        lon1_rad = lon1 * math.pi /180
        lon2_rad = lon2 * math.pi /180
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        earth_R = 6371                                   # Earth's radius in km
        a = math.pow(math.sin(delta_lat/2),2) + math.cos(lat1_rad) * math.cos(lat2_rad) * math.pow(math.sin(delta_lon/2),2);
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a));
        dist = earth_R * c * 1000; # in meters  
        return dist
	
# pdist([[1,1],[3,3]])
# array([ 2.82842712])


def generate_next4(curx, cury):
#generate the next four location based on current location
# return a list of four loc
	target_list=[]
	target_list.append([curx + 15, cury])
	target_list.append([curx - 15, cury])
	target_list.append([curx , cury +15])
	target_list.append([curx , cury -15])
	return target_list
	
def  predict_rssi (target_list):
#predict the rssi value for the list of target
	rssi_target_pred=[]
	for tt in target_list:
		rssi_tmp = rssi_xygps(tt, target_act)	

		rssi_target_pred.append(rssi_tmp)		
		#rssi_target_pred.append(rssi_obs[0])		
	return rssi_target_pred

parser = argparse.ArgumentParser(description='Commands vehicles using vehicles.simple_goto.')
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

vehicles=[]
if not connection_string:
    import dronekit_sitl
    numinst=2
    ports=[10001,1002]
    lats=[-35.361, -35.371]
    lons=[149.167, 149.177]
    sitlv = dronekit_sitl.start_swarm(numinst, lats, lons, ports)
    vid=1
    for sitl in sitlv:
	connection_string = sitl.connection_string()
	# Connect to the Vehicle
	print('Connecting to vehicles on: %s' % connection_string)
	vehicle = connect(connection_string, wait_ready=True)
	out = MAVConnection(gcs, source_system=vid)
#out2 = MAVConnection(sys.argv[3], source_system=253)
	vehicle._handler.pipe(out)
	out.start()
	vehicles.append(vehicle)
	vid =vid+1

#vehicles2._location  = LocationGlobalRelative(-35.361354, 149.167218, 20)
print ("local to gps sanity check: gps")
print(vehicles[0]._location._lat)
print(vehicles[0]._location._lon)
print("v2 gps:")
print(vehicles[1]._location._lat)
print(vehicles[1]._location._lon)
print ("local to gps sanity check: v1 gps recovered")
(v1x,v1y)=gps_to_local(vehicles[0]._location._lat, vehicles[0]._location._lon)
(v1_lat_rec, v1_lon_rec)= local_to_gps(v1x, v1y)
print((v1_lat_rec, v1_lon_rec))

def arm_and_takeoff(vehicles, aTargetAltitude):
    """
    Arms vehicles and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicles.is_armable:
        print(" Waiting for vehicles to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicles.mode = VehicleMode("GUIDED")
    vehicles.armed = True

    # Confirm vehicles armed before attempting to take off
    while not vehicles.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicles.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicles reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicles.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicles.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(vehicles[0], 10)
arm_and_takeoff(vehicles[1], 10)

print("Set default/target airspeed to 3")
vehicles[0].airspeed = 10
vehicles[1].airspeed = 10

time.sleep(2)
print("v#1 Going towards v#2 point for 30 seconds ...")
target_act=[ vehicles[1]._location._lat, vehicles[1]._location._lon]
rssi_obs=[]
while True:
	v2_lat= vehicles[1]._location._lat
	v2_lon= vehicles[1]._location._lon
	v1_lat= vehicles[0]._location._lat
	v1_lon= vehicles[0]._location._lon
	point1 = LocationGlobalRelative(v2_lat, v2_lon, 20)
	#vehicles.simple_goto(point1)
	rssival, v1_xy = rssi([v1_lat,v1_lon],[v2_lat, v2_lon])
	rssi_obs.append([v1_xy[0], v1_xy[1], rssival])

	next_target_list=generate_next4(v1_xy[0], v1_xy[1])
	print (next_target_list)
	#generate four moving candidate, return a list

	rssi_targetlist = predict_rssi (next_target_list)
	#return the list with corresponding rssi value
	
	maxrssi=max(rssi_targetlist)
	maxrssiind = rssi_targetlist.index(maxrssi)
	target=next_target_list[maxrssiind]
	
	print("\n\ntarget  %f %f.." %(target[0], target[1]))
	targetgps = local_to_gps(target[0], target[1])
	print("targetgps  %f %f.." %(targetgps[0], targetgps[1]))
# LocationGlobalRelative(-35.361354, 149.167218, 20)
	vehicles[0].simple_goto(LocationGlobalRelative(targetgps[0],targetgps[1],20))

	print("v#1 curr loc ...")
	print(v1_xy)
	print((vehicles[0]._location._lat, vehicles[0]._location._lon))
	print("rssi value %f\n\n" %rssival)
	time.sleep(5)

# sleep so we can see the change in map
time.sleep(30)

#print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")


print("Returning to Launch")
vehicles[0].mode = VehicleMode("RTL")
vehicles[1].mode = VehicleMode("RTL")

# Close vehicles object before exiting script
print("Close vehicles object")
vehicles[0].close()
vehicles[1].close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
    sitl2.stop()
