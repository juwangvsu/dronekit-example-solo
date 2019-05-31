#!/usr/bin/env python

import rospy
import sys
import time
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from rssi_msgs.msg import rssi 
import math
import roslib; roslib.load_manifest('actionlib_tutorials')
import actionlib
import actionlib_tutorials.msg
import move_base_msgs.msg

def turnz(ang_rad):
    	cmd_vel_msg = Twist()
#    cmd_vel_msg.linear.x  = msg.axes[linear_axis]   * linear_scale
        cmd_vel_msg.angular.z = 3.14 * (45.0/180) # 45 deg/sec
    	cmd_vel_pub.publish(cmd_vel_msg)
	turntime = abs(ang_rad)/cmd_vel_msg.angular.z
	print "turning time:"
	print turntime
	time.sleep(turntime)
        cmd_vel_msg.angular.z = 0 # stop  
    	cmd_vel_pub.publish(cmd_vel_msg)

# move forward by x meters
def forwardx(dist_meter):
    	cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x  = 0.5 # 0.5 meters /sec
        cmd_vel_msg.angular.z = 0 
    	cmd_vel_pub.publish(cmd_vel_msg)
	turntime = dist_meter/cmd_vel_msg.linear.x
	time.sleep(turntime)
        cmd_vel_msg.linear.x = 0 # stop  
    	cmd_vel_pub.publish(cmd_vel_msg)

class RssiRecord:
	def __init__(self, nodeid=None, rssival=None,ctpseq=None, timestamp=None,x=None,y=None ):
		self.nodeid=nodeid 
		self.rssival=rssival
		self.ctpseq=ctpseq
		self.timestamp=timestamp
		self.x=x  
		self.y=y 
    	def __repr__(self):
         	return "Test()"
        def __str__(self):
		return "%s %s %s %s %s " % (self.timestamp, self.ctpseq, self.rssival, self.x, self.y)
            	
# call action server to navigate to a point about one meter in front
def forward_x_meter_client(x):
  	global curr_pose
    	client = actionlib.SimpleActionClient('forward_x_meter', move_base_msgs.msg.ForwardXMeterAction)
    	client.wait_for_server()
	goal = move_base_msgs.msg.ForwardXMeterGoal(dist=x)
	print curr_pose
    	client.send_goal(goal)
    	print 'waiting for result...'
    	client.wait_for_result()
    	print 'got result'
    	return client.get_result()  

def dist_map(x0, y0, x1, y1):
	return pow((x0-x1),2)+pow((y0-y1),2)
	
def poseCB(msg):
  global firstpose
  global curr_pose
  global last_pose # last recorded pose that an rssi probability is calculated
  global startrssi
  global cmd_vel_pub
  global rssilist
  global rssilist_map
  global linear_axis
  global linear_scale
  global rotation_axis
  global rotation_scale
  global curr_ptu_angle
  global startrssi
  global resetrssi
  global rssi_index
  global abort_nav_button

  rssilist_current=rssilist_map # decide which rssilist will be used, raw list or intrepeded map

  curr_pose=msg
  if firstpose:
	last_pose=msg
	firstpose=False
  print curr_pose.pose.pose.position,"\n"
  mindist = 50
  minrssiitem=rssilist_current[1]
  x= curr_pose.pose.pose.position.x
  y= curr_pose.pose.pose.position.y
  
  for rssiitem in rssilist_current:
	m_dist = dist_map(x,y, rssiitem.x, rssiitem.y)
	if (m_dist<mindist):
		mindist = m_dist
		minrssiitem = rssiitem

  print minrssiitem
  rssi_msg = rssi()
#    cmd_vel_msg.linear.x  = msg.axes[linear_axis]   * linear_scale
  rssi_msg.rssi_val = minrssiitem.rssival
  rssi_msg.from_id = minrssiitem.nodeid
  rssi_msg.ctp_seq = minrssiitem.ctpseq
  rssi_pub.publish(rssi_msg)

  lastx=last_pose.pose.pose.position.x
  lasty=last_pose.pose.pose.position.y
  if dist_map(x,y, lastx, lasty) > 2:
# calculate new target position, and new movement goal
	last_pose=curr_pose
	target_pos = target_pos_update(rssi_msg.rssi_val, curr_pose)
	goal_update(target_pos)

#  execute_new_goal()

def target_pos_update(rssival, pos):
	return (0,0)

def goal_update(target_pos):
	global got_new_goal
	global new_goal
	got_new_goal = True 
	new_goal = target_pos

def execute_new_goal():
	global got_new_goal
	global new_goal
	print "execute new goal"
	forwardx(1)

def mission():
	while(True):
		execute_new_goal()
		time.sleep(1)


def follownode(nodeid):
#enable rssi reception, scan ptu, turn off rssi 
#receiving, search the angle with highest rssi
#turn robot accordingly, then move forward by
#1 meter, or call a move_base service "forward_x_meter"
#which will check costmap and decide a feasible goal x meter in front
#of robot
  global startrssi
  global resetrssi
  while (True):
    resetrssi = True 
    startrssi = True 
    startrssi = False
    if len(rssilist) ==0:
#	turnz(3.14/2)
	time.sleep(5)
	#forwardx(1)
	# service call to move_base
    else:
	print "total rssi record is: %d" %len(rssilist)
    	curr_high=rssilist[0].rssival
    	curr_high_angle = rssilist[0].x
    	for rssiitem in rssilist:
		print rssiitem.rssival
		if rssiitem.rssival > curr_high:
			print "new high at %f degree" %rssiitem.x
			curr_high= rssiitem.rssival
			curr_high_angle = rssiitem.x

        #print "PTU set to highest rssi at %f degree" %curr_high_angle
	time.sleep(3)

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

def dumprssilist(m_rssilist):
	print "total number of rssi record %d" %len(m_rssilist)
	for i in range(0,100):
		print m_rssilist[i]

def loadrssi_map_f():
	global rssilist_map
    	rssimapf_name = "/media/student/NewVolume/rssi_gps_data_matlab/0115_2016/rssi_map_f.mat"
    	rssimapf = open(rssimapf_name)
    	rssilist_map = []

	for xi in range(1,34): # loop 33 time, excluding 34
        #print line,
		line = rssimapf.readline()
		line = line.strip()
		columns = line.split()
		print len(columns)
		for xfield in columns:
			rssirec = RssiRecord(0,0, 0,0,float(xfield),0)
			rssilist_map.append(rssirec)

	print "total rssimap record "
	print len(rssilist_map)
	for yi in range(1,34):
        #print line,
		line = rssimapf.readline()
		line = line.strip()
		columns = line.split()
		columnind=0
		for yfield in columns:
			#print " rssimap ind "
			#print (yi-1)*33+columnind

			rssilist_map[(yi-1)*33+columnind].y=float(yfield)
			columnind = columnind +1

	for zi in range(1,34):
        #print line,
		line = rssimapf.readline()
		line = line.strip()
		columns = line.split()
		columnind=0
		for zfield in columns:
			rssilist_map[(zi-1)*33+columnind].rssival=float(zfield)
			columnind = columnind +1
	

if __name__ == '__main__':
    rospy.init_node('rssi_sim')
    global firstpose

    global rssi_pub
    global cmd_vel_pub
    global rssilist
    global rssilist_map # this from rssi_map_f.mat

    global rssi_index
    global curr_ptu_angle
    global resetrssi 
    global startrssi 
    global gpsrefok;
    global gps_ref_lat;
    global gps_ref_lon;

    firstpose = True # firstpose become false after receive the first pose msg
    gpsrefok =0;
    resetrssi = True 
    startrssi = False 
    curr_ptu_angle=-10
    rssi_index=0;
    rssifilename = "/media/student/New Volume/rssi_gps_data_matlab/0115_2016/rssi_dronepose_01151436.txt"
    gpsfilename = "/media/student/New Volume/rssi_gps_data_matlab/0115_2016/gpsfix_01151436.txt"
    rssif = open(rssifilename)
    gpsfixf = open(gpsfilename)
    gpsfixf.readline()
    rssif.readline()
# get the reference gps position, the first line of the master log, 
# the relative location will be different if the reference line 
# use the gpsfixf 
    line = rssif.readline()
    line = line.strip()
    columns = line.split()
    print columns[1],columns[2]
    gps_ref_lat =float(columns[10])# if using gpsfixf columns[1]
    gps_ref_lon =float(columns[11])# if using gpsfixf columns[2]
    rssilist = []
    for line in rssif:
        #print line,
	line = line.strip()
	columns = line.split()
	nodeid = columns[1]
	rssival = columns[2]
	ctpseq = columns[3]
	timestamp = columns[9]
	lat2 = float(columns[10])
	lon2 = float(columns[11])

        lat3 = gps_ref_lat
        lon3 = lon2
        x= math.copysign(1, lon2-gps_ref_lon) * haversine(gps_ref_lat, gps_ref_lon, lat3,lon3)
        y= math.copysign(1, lat2-gps_ref_lat) *haversine(lat3, lon3, lat2, lon2)
	rssirec = RssiRecord(nodeid,rssival, ctpseq,timestamp,x,y)
	#print rssival, x, y
	print rssirec
	rssilist.append(rssirec)
    print rssilist[1]
    loadrssi_map_f() #load rssi map from .mat file

    dumprssilist(rssilist_map)
    rssi_pub = rospy.Publisher("rssi_msg", rssi, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/Pioneer3AT/cmd_vel", Twist)
    rospy.Subscriber("/Pioneer3AT/pose", Odometry, poseCB)
    #follownode(3)
    mission()
    rospy.spin()
#    scan_angles(
#            float(sys.argv[1]),
#            float(sys.argv[2]),
#            float(sys.argv[3]),
#            int(sys.argv[4]))
