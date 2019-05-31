#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
Ju Wang, Virginia State University

simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

this version add a udp thread to handle communication with controller, interface should match with
georgo hwang's controller: udp ports # are 14551, 14552, command format: state msg format, 
				local command udp port 14551
			        remote port where controller will be listening 14552
	7/20/17: test gps_input message
	7/17/17: two vehadp thread are created:
			(1) cmd thread wait for cmd (blocking), perform action, and sendback reply. 
				currentlu dummy action and reply.
			(2) state pub thread publish veh data 0.5 Hz. currently dummy data
			(3) to test the two threads, use udpclient.py and udprcv.py under solo
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from dronekit.mavlink import MAVConnection
import threading
# Set up option parsing to get connection string
import argparse
import socket
from pymavlink import mavutil, mavwp

global currstate, statelist, vehicle, dbgflag, infoflag, localwp
global velocity_x, velocity_y, velocity_z, localtwist_duration

localwp=None
dbgflag=False
infoflag=False
currstate='idle'
statelist=['idle', 'arm', 'wp', 'land', 'shutdown']
vehicle=None
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--gcs',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--simulation',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")

args = parser.parse_args()

connection_string = args.connect
gcs = args.gcs
simulation=args.simulation
sitl = None

#*******************************************************************************

def send_gps_input_mavlink():

    global vehicle
# for more message type, see: pymavlink/dialects/v10/common.py, ardupilotmega.py

        #def gps_input_encode(self, time_usec, gps_id, ignore_flags, time_week_ms, time_week, fix_type, lat, lon, alt, hdop, vdop, vn, ve, vd, speed_accuracy, horiz_accuracy, vert_accuracy, satellites_visible):
    msg = vehicle.message_factory.gps_input_encode(
        0,       # time_boot_ms (not used)
        1, 0, #gps_id, ignore_flags  
	1234, 567, #time_week_ms, time_week,
	3, -5.371, 9.177, 5, # fix type, lat, lon, alt 
	1, 1, #hdop, vdop
	0,0,0, #vn, ve, vd
	0.1, 0.1, 0.1,11 #speed_accuracy, horiz_accuracy, vert_accuracy, sate_visible
	)

    for x in range(0,100):	
	if vehicle and infoflag:
	  print("simple_goto: send a gps input msg")
	#print(msg)
        vehicle.send_mavlink(msg)
        time.sleep(1)

# def gps_raw_int_encode(self, time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible):

    msg = vehicle.message_factory.gps_raw_int_encode(
        0,       # time_boot_ms (not used)
	3, -5.371, 9.177, 5, # fix type, lat, lon, alt 
	0,0, #eph, epv, 
	4, 0.1, 12 #speed_, cog,  sate_visible
	)

    for x in range(0,20):	
	print("simple_goto: send a gps_raw_int msg")
	#print(msg)
        vehicle.send_mavlink(msg)
        time.sleep(1)

#def command_int_encode(self, target_system, target_component, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    msg = vehicle.message_factory.command_int_encode(
        1,       # target_sys 
	1, frame, 21, 1, # target_component, frame, cmd, current 
	0, #autocontinue,  
	4, 0.1, 12,0,0,0,0 #param1, 2, 3, 4, x, y, z, 
	)

    for x in range(0,20):	
	print("simple_goto: send a command_int msg")
        vehicle.send_mavlink(msg)
        time.sleep(1)

#*******************************************************************************
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    global vehicle
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        #mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
#*******************************************************************************

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


#start vehicle adapter udp thread to interact with george hwang's controller

sockaddr_local=("",14551)
sockaddr_remote=("localhost",8889)
#sockaddr_remote=("10.1.1.11",8889)

#*******************************************************************************
def vehadp_thread_statepub():
    global vehicle, dbgflag, infoflag
    vehlat=0
    vehlon=0
    '''vehadp state pub  thread'''
    print("vehadp thread start publishing state...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
	vehstatmsg='pos: lat=0.0, lon=0.0'
        if vehicle:
		if dbgflag:
      			print(" Altitude: ", vehicle.location.global_relative_frame.alt)
      		vehlat=vehicle.location.global_relative_frame.lat
      		vehlon=vehicle.location.global_relative_frame.lon
		vehstatmsg='pos: lat='+str(vehlat)+', lon='+str(vehlon)
        sock.sendto(vehstatmsg, sockaddr_remote)
	if dbgflag:
		print ("vehadp thread pub msg: %s" %vehstatmsg)
	if vehicle and infoflag:
		print( "Local Location: %s" % vehicle.location.local_frame)    #NED
		print( "Global Location: %s" % vehicle.location.global_relative_frame)    #NED
	time.sleep(1)

#*******************************************************************************
def vehadp_thread_cmd():
    global currstate, dbgflag, localwp, infoflag
    global velocity_x, velocity_y, velocity_z, localtwist_duration
    '''vehadp cmd thread'''
    print("vehadp thread start...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(sockaddr_local)

    while True:
        msg = sock.recvfrom(1024)
	print (msg)
	print ("vehadp thread received msg")
        remoteip=msg[1][0]
        remoteport=msg[1][1]
	cmdmsg=msg[0].strip().split(':')
	print (msg[0])
	print (cmdmsg[0])
	if cmdmsg[0]=='arm':
		print ("vehadp thread received arm")
		if len(cmdmsg)==2:
		  if cmdmsg[1]=='1':
			print ("vehadp thread arming")
			currstate='arm'
		  elif cmdmsg[1]=='0':
			print ("vehadp thread disarming")
			currstate='disarm'
		else:
			print(" arm formt: localtwist:10:10")
	elif cmdmsg[0]=='land':
		print ("vehadp thread received land")
		currstate='land'
	elif cmdmsg[0]=='gpsinput':
		print ("vehadp thread received gpsinput")
		currstate='gpsinput'
	elif cmdmsg[0]=='wp':
		print ("vehadp thread received wp")
		currstate='wp'
	elif cmdmsg[0]=='localwp':
		if len(cmdmsg)==3:
			localwp=LocationLocal(int(cmdmsg[1]),int(cmdmsg[2]),0)
			print ("vehadp thread received localwp")
			print(localwp)
			currstate='localwp'
		else:
			print(" localwp formt: localtwist:10:10")
	elif cmdmsg[0]=='localtwist':
		if len(cmdmsg)==5:
			velocity_x=int(cmdmsg[1])
			velocity_y=int(cmdmsg[2])
			velocity_z=int(cmdmsg[3])
			localtwist_duration=int(cmdmsg[4])
			print ("vehadp thread received localtwist")
			currstate='localtwist'
		else:
			print(" localtwist formt: localtwist:10:10:4:10")
	elif cmdmsg[0]=='shutdown':
		print ("vehadp thread received shutdown")
		currstate='shutdown'
	elif cmdmsg[0]=='info':
		print ("vehadp thread received info")
		currstate='info'
		infoflag=not infoflag
	elif cmdmsg[0]=='rtl':
		print ("vehadp thread received rtl")
		currstate='rtl'
	elif cmdmsg[0]=='guided':
		print ("vehadp thread received dbg")
		currstate='guided'
	elif cmdmsg[0]=='dbg':
		print ("vehadp thread received dbg")
		dbgflag=not dbgflag


        sock.sendto('cmd recved',(remoteip, remoteport))
                #print( "use 192.168.1.255 or similar for bcast address, not 255.255.255.255\nespecially if localhost")`
                #s.sendto('this is testing',('255.255.255.255',14550))

t_vehadp = threading.Thread(target=vehadp_thread_cmd, name='vehicleadapter')
t_vehadp.daemon = True
t_vehadp.start()
infoflag=True
t_vehadp_state = threading.Thread(target=vehadp_thread_statepub, name='vehicleadapterstatepub')
t_vehadp_state.daemon = True
t_vehadp_state.start()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print (gcs)
print (gcs==None)
if not gcs==None:
	out = MAVConnection(gcs, source_system=1)
	vehicle._handler.pipe(out)
	out.start()

#*******************************************************************************
def arm_and_takeoff(aTargetAltitude):
    global currstate
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks\n, diff from simple_goto.py: no initialzation check, start with stablized mode")
    # Don't try to arm until autopilot is ready
#   arming check in solo not consist with dronekit, tbi, jwang 7/14/17
    print("simulation: %s"%simulation)
    if simulation: #true if it is not None
      print("simulation is yes")
      while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    if not simulation: 
	print("real solo, enter stablize mode")
    	vehicle.mode = VehicleMode("STABILIZE")#guided mode need gps, stablize does not
    else:
	print("simulated solo, enter guide mode, stablized mode will auto disarm somehow???")
    	vehicle.mode = VehicleMode("GUIDED")

    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    print("start with guided mode, \nif no gps, solo will automatic goto alt-hold mode, and disarm")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
#    while True:
#        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
#        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
#            print("Reached target altitude")
#            break
#        time.sleep(1)

#*******************************************************************************
#main event loop:
#global keeplooping
#keeplooping=True
while True:
	if dbgflag:
		print("curr state is: %s"%currstate)
		print("curr mode is: %s"% vehicle.mode)
#	print("keeplooping: %d"%keeplooping)
	if currstate=='idle': 
	  	if dbgflag:
		  print("main thread enter idle state")
		time.sleep(1)
	elif currstate=='armed':
		time.sleep(1)
	elif currstate=='disarm':
    		#vehicle.mode = VehicleMode("STABILIZE")
    		vehicle.mode = VehicleMode("LAND")
		time.sleep(3)
    		#vehicle.mode = VehicleMode("LAND")
    		vehicle.armed = False
		time.sleep(1)
		currstate='idle'
		#currstate='land'
	elif currstate=='arm':
		print("main thread enter arm state")
		arm_and_takeoff(2)
		print("Set default/target airspeed to 3")
		vehicle.airspeed = 3
		currstate='armed'

	elif currstate=='wp':
		print("main thread enter wp state")
		print("Going towards first point for 30 seconds ...")
		point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
		vehicle.simple_goto(point1)

# sleep so we can see the change in map
		time.sleep(30)
		print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
		point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
		vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
		time.sleep(30)
		currstate='rtl'
	elif currstate=='localwp':
		print("main thread enter localwp state, it will go to a local relative position")
		curr_localposition = vehicle.location.local_frame 
		if curr_localposition.north: #if valid, north will not be None
			target_localposition = LocationLocal(curr_localposition.north + localwp.north, curr_localposition.east + localwp.east, curr_localposition.down)
			print("target local position")
			print(target_localposition)
			print("current local position")
			print(curr_localposition)
		else:
			print ("curr localposition None, arm first ...")
		time.sleep(1)
	elif currstate=='localtwist':
		print("main thread enter localtwist state, it will go to a local relative position")
		send_ned_velocity(velocity_x, velocity_y, velocity_z, localtwist_duration)
		currstate='idle'

	elif currstate=='rtl':
		print("Returning to Launch")
		vehicle.mode = VehicleMode("RTL")
		time.sleep(1)
		currstate='idle'
	elif currstate=='land':
		print("land")
		vehicle.mode = VehicleMode("LAND")
		time.sleep(1)
		currstate='idle'
	elif currstate=='gpsinput':
		print("send a gps input to mavlink")
		send_gps_input_mavlink()
		time.sleep(1)
		currstate='idle'
	elif currstate=='guided':
		print("Returning to Launch")
		vehicle.mode = VehicleMode("GUIDED")
		time.sleep(1)
		currstate='idle'

	elif currstate=='shutdown':
# Close vehicle object before exiting script
		print("Close vehicle object")
		vehicle.close()
		time.sleep(1)
		break
	elif currstate=='info':
		#info printing is handled by the thread and infoflag
        	#print(" Altitude: ", vehicle.location.global_relative_frame.alt)
	#	print( "Local Location: %s" % vehicle.location.local_frame)    #NED
		time.sleep(1)
        # Break and return from function just below target altitude.
# Shut down simulator if it was started.
if sitl:
    sitl.stop()
