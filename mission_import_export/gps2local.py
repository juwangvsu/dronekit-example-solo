#!/usr/bin/env python
# 10/1/2015 ju wang
#	this script listen to rssi, pose, fix topic and save then in data file rssi_dronepose.txt

import sys
import roslib
import rospy
import math
import tf
import sys, termios, tty, select, os
import geometry_msgs.msg
import  nav_msgs.msg
import rssi_msgs.msg
import sensor_msgs.msg
import tum_ardrone.msg
from datetime import datetime
import ardrone_autonomy.msg

from tf.transformations import euler_from_quaternion
#this script is print out the tf from map to odom 

def gps2localconv(gps_latitude, gps_longitude):
#gpsfix callback
	global gpsfile;
	global gps_ref_lat;
	global gps_ref_lon;
# first fix, save the reference point
	print 'drone /fix', gps_latitude, gps_longitude, '\n'
	gpsfile.write("\t")
	gpsfile.write(str(gps_latitude))
	gpsfile.write("\t")
	gpsfile.write(str(gps_longitude))
	gpsfile.write("\t")

	lat2 =gps_latitude
	lon2 = gps_longitude
	lat3 = gps_ref_lat
	lon3 = lon2
 	print(gps_ref_lat,gps_ref_lon,lat3, lon3,lat2, lon2)	
	x= math.copysign(1, lon2-gps_ref_lon) * haversine(gps_ref_lat, gps_ref_lon, lat3,lon3)
	y= math.copysign(1, lat2-gps_ref_lat) *haversine(lat3, lon3, lat2, lon2)

	gpsfile.write(str(x))
	gpsfile.write("\t")
	gpsfile.write(str(y))
	gpsfile.write("\n")
	gpsfile.flush()

def haversine(lat1, lon1, lat2, lon2):
#calculate dist between two points based on gps lat and lon_lon
	# Convert all decimal degrees to radians
	#test case: 37.2398256	-77.4185491
 	#37.239869	-77.4183346 , dist 19.6 meters
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
     
def gps2local():
	global gpsfile;
	global gps_ref_lat;
	global gps_ref_lon;
	missionfilename = 'mpmission.txt'
	missionfile= open(missionfilename, "r") #gps fix data

	gpsfilename = 'mpmission_local.txt'
	gpsfile= open(gpsfilename, "wb") #gps fix data

# Read and ignore header lines
	header1 = missionfile.readline()
	wplist=[]
# Loop over lines and extract variables of interest
	for line in missionfile:
    		line = line.strip()
    		columns = line.split()
		#print(columns)
    		columns = line.split()
    		lat = float(columns[8])
    		lon = float(columns[9])
    		print(lat, lon)
		wplist.append((lat,lon))
	print wplist
	gps_ref_lat=wplist[0][0] 		
	gps_ref_lon=wplist[0][1]
	for wp in wplist:
		gps2localconv(wp[0], wp[1]) 		
	missionfile.close()

if __name__ == '__main__':
	global settings
	# termios cause problem when run from gpsd-start, so comment out
	#settings = termios.tcgetattr(sys.stdin)
	print 'Number of arguments:', len(sys.argv), 'arguments.'
	print 'Argument List:', str(sys.argv)
	gps2local()
