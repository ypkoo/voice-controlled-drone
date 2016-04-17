"""
By YP KOO

Ground control station code for voice controlled drone.
"""

import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import json, math, time, sys, ssl
import logging, logging.handlers, traceback
from gcs_mqttc import mqttc, host, port, topic
import argparse 

is_subscribed = False






""" Callback functions for mqtt client """
def on_subscribe(mqttc, obj, mid, granted_qos):
	print("Subscribed: "+str(mid)+" "+str(granted_qos)+"data"+str(obj))

	global is_subscribed
	is_subscribed = True

def on_message(mqttc, obj, msg):
	print("Received message from topic: "+msg.topic+" | QoS: "+str(msg.qos)+" | Data Received: "+str(msg.payload))

	data = json.loads(str(msg.payload))

	intent_name = data['name']

	if intent_name == "Go":
		do_go(data)
	elif intent_name == "Command":
		do_command(data)
	elif intent_name == "Turn":
		do_turn(data)



def do_go(data):
	direction = data["slots"]["Direction"]["value"]
	distance = int(data["slots"]["Distance"]["value"])
	unit = data["slots"]["Unit"]["value"]  # for now this is hard coded at meters.

	# convert distance to meters
	if unit in ["feet", "foot"]:
		distance = 0.3048 * distance
	elif unit in ["yard", "yards"]:
		distance = 0.9144 * distance

	my_yaw = vehicle.attitude.yaw

	# goto(dNorth, dEast, gotoFunction=vehicle.commands.goto):
	if direction in ["forward", "forwards", "straight", "straight ahead"]:
		my_direction = my_yaw
		my_xy = calc_xy(distance, my_direction)
		goto(my_xy[0], my_xy[1])
	elif direction in ["back", "backward", "backwards"]:
		my_direction = (my_yaw + math.pi) % math.pi
		my_xy = calc_xy(distance, my_direction)
		goto (my_xy[0], my_xy[1])
	elif direction == "right":
		my_direction = (my_yaw + (math.pi / 2)) % math.pi
		my_xy = calc_xy(distance, my_direction)
		goto (my_xy[0], my_xy[1])
	elif direction == "left":
		my_direction = (my_yaw + (math.pi * 1.5)) % math.pi  # not "- math.pi / 2"?
		my_xy = calc_xy(distance, my_direction)
		goto (my_xy[0], my_xy[1])
	elif direction == "up":  #  goto_position_target_local_ned(north, east, down):
		change_altitude(distance)
	elif direction == "down":
		change_altitude(distance * -1)

def normalize_yaw(angle):      # change it from negative pi to positive pi, to more normalized zero to 2*pi
	if angle < 0:       # couldn't we also just do angle + math.pi???
		angle = (2 * math.pi) + angle       #    angle = abs(angle)  it's PLUS, because we're adding a Negative number...
	return angle

def calc_xy (distance, angle, units="radians"):
	if units != "radians":
		angle = math.radians(angle)
	angle = normalize_yaw(angle)
	x = distance * (math.cos(angle))
	y = distance * (math.sin(angle))
	return (round (x, 1) , round(y, 1))

def convert_yaw_to_degrees(angle):
	angle = normalize_yaw(angle)
	return math.degrees(angle)

def do_command(data):
	task = data['slots']['Task']['value']

	if task == "launch":
		arm_and_takeoff()
	elif task == "land":
		vehicle.mode = VehicleMode("LAND")
		vehicle.flush()

def do_turn(data, degrees=90):
	direction = data["slots"]["Direction"]["value"]

	# rotation = 0
	if direction == "right":
		rotation = 1
	elif direction == "left":
		rotation = -1
	elif direction == "around":
		degrees=180
		rotation = 1
	else:
		return
	condition_yaw(degrees, rotation, True)

def arm_and_takeoff(aTargetAltitude = 5):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print "Basic pre-arm checks"
	# Don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		print " Waiting for vehicle to initialise..."
		time.sleep(1)

	print "Arming motors"
	# Copter should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True    

	# Confirm vehicle armed before attempting to take off
	while not vehicle.armed:      
		print " Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
	#  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print " Altitude: ", vehicle.location.global_relative_frame.alt 
		#Break and return from function just below target altitude.        
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
			print "Reached target altitude"
			break
		time.sleep(1)

def condition_yaw(heading, rotate=1, relative=False):
	"""
	Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

	This method sets an absolute heading by default, but you can set the `relative` parameter
	to `True` to set yaw relative to the current yaw heading.

	By default the yaw of the vehicle will follow the direction of travel. After setting
	the yaw using this function there is no way to return to the default yaw "follow direction
	of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

	For more information see:
	http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
	"""

	# rot = rotation:  -1 = ccw, 1 = cw
	if relative:
		is_relative = 1  # yaw relative to direction of travel
	else:
		is_relative = 0  # yaw is an absolute angle
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,  # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
		0,  # confirmation
		heading,  # param 1, yaw in degrees
		0,  # param 2, yaw speed deg/s
		rotate,  # param 3, direction -1 ccw, 1 cw
		is_relative,  # param 4, relative offset 1, absolute angle 0
		0, 0, 0)  # param 5 ~ 7 not used
	# send command to vehicle
	vehicle.send_mavlink(msg)
	vehicle.flush()

def goto(dNorth, dEast):
	"""
	Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
	The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
	the target position. This allows it to be called with different position-setting commands. 
	By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
	The method reports the distance to target every two seconds.
	"""
	
	currentLocation = vehicle.location.global_relative_frame
	targetLocation = get_location_metres(currentLocation, dNorth, dEast)
	targetDistance = get_distance_metres(currentLocation, targetLocation)
	vehicle.simple_goto(targetLocation)
	
	#print "DEBUG: targetLocation: %s" % targetLocation
	#print "DEBUG: targetLocation: %s" % targetDistance

	while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
		#print "DEBUG: mode: %s" % vehicle.mode.name
		remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
		print "Distance to target: ", remainingDistance
		if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
			print "Reached target"
			break;
		time.sleep(2)



def change_altitude(meters_up):     # flip the NED (Down) to UP, before you pass in the value.
	# Set mode to guided - this is optional as the goto method will change the mode if needed.
	vehicle.mode = VehicleMode("GUIDED")
	# Set the location to goto() and flush()
	a_location = Location(vehicle.location.lat, vehicle.location.lon, meters_up, is_relative=True)
	vehicle.commands.goto(a_location)
	vehicle.flush()


def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
	specified `original_location`. The returned LocationGlobal has the same `alt` value
	as `original_location`.
	The function is useful when you want to move the vehicle around specifying locations relative to 
	the current vehicle position.
	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius = 6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	if type(original_location) is LocationGlobal:
		targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
	elif type(original_location) is LocationGlobalRelative:
		targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
	else:
		raise Exception("Invalid Location object passed")
		
	return targetlocation;




def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two Location objects.

	This method is an approximation, and will not be accurate over large distances and close to the
	earth's poles. It comes from the ArduPilot test code:
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def goto_position_target_local_ned(north, east, down):
	"""
	Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
	location in the North, East, Down frame.

	It is important to remember that in this frame, positive altitudes are entered as negative
	"Down" values. So if down is "10", this will be 10 metres below the home altitude.

	At time of writing the method ignores the frame value and the NED frame is relative to the
	HOME Location. If you want to specify the frame in terms of the vehicle (i.e. really use
	MAV_FRAME_BODY_NED) then you can translate values relative to the home position (or move
	the home position if this is sensible in your context).

	For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_LOCAL_NED

	See the above link for information on the type_mask (0=enable, 1=ignore).
	At time of writing, acceleration and yaw bits are ignored.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,  # time_boot_ms (not used)
		0, 0,  # target system, target component
		mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
		0b0000111111111000,  # type_mask (only positions enabled)
		north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
		0, 0, 0,  # x, y, z velocity in m/s  (not used)
		0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# send command to vehicle
	vehicle.send_mavlink(msg)
	vehicle.flush()

""" Location update callback function """
def location_callback(self, attr_name, value):
	print "Location (Global): ", value

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Commands vehicle using Amazon Echo.')
	parser.add_argument('--connect', 
					   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
	args = parser.parse_args()

	if args.connect == "sitl":
		connection_string = 'tcp:127.0.0.1:5760'
		print "Start simulator (SITL)"
		from dronekit_sitl import SITL
		sitl = SITL()
		sitl.download('copter', '3.3', verbose=True)
		sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
		sitl.launch(sitl_args, await_ready=True, restart=True)
	elif args.connect == "solo":
		connection_string = 'udpin:0.0.0.0:14550'
	else:
		print "wrong argument: " + args.connect
		sys.exit()

	# Connect to the Vehicle.
	print "Connecting to vehicle on: " + connection_string
	vehicle = connect(connection_string, wait_ready=True)

	# Add a callback `location_callback` for the `global_frame` attribute.
	vehicle.add_attribute_listener('location.global_frame', location_callback)

	mqttc.on_subscribe = on_subscribe
	mqttc.on_message = on_message
	mqttc.connect(host, port)
	mqttc.loop_start()

	while not is_subscribed:
		print "Waiting for subscribing..."
		mqttc.subscribe(topic, qos=1)
		time.sleep(1)


	mqttc.loop_stop()

	while True:
		time.sleep(1)
		# print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
		mqttc.loop()