#!/usr/bin/python

"""python groundstation for 3DR drones to be controlled by Amazon Echo"""

import paho.mqtt.client as mqtt
#from droneapi.lib import VehicleMode, Location
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import json, math, time, sys, ssl
import logging, logging.handlers, traceback

cert_path = "../EchoDroneControl/awsCerts/"
host = "A30ITWQ5LJOT4V.iot.us-east-1.amazonaws.com"
topic = "$aws/things/GroundStation01/shadow/update"
root_cert = cert_path + "root-CA.crt"
cert_file = cert_path + "3480a0ba5b-certificate.pem.crt"
key_file = cert_path + "3480a0ba5b-private.pem.key"

globalmessage = ""  # to send status back to MQTT -- arming, no gps lock, etc.

client = mqtt.Client(client_id="echodronectl.py")
client.tls_set(root_cert,
               certfile = cert_file,
               keyfile = key_file,
               tls_version=ssl.PROTOCOL_SSLv23,
               ciphers=None)
               #tls_version=ssl.PROTOCOL_TLSv1_2,

client.connect(host, 8883, 60)


#api = local_connect()
vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True)

logger = logging.getLogger('echodronectl')


def do_command(data):  #   {"name":"CommandIntent","slots":{"Task":{"name":"Task","value":"launch"}}}

    task = str(data["slots"]["Task"]["value"])
    logger.info("TASK = " + task)
    global globalmessage

    if task == "launch":
        globalmessage = "executing command launch"
        print globalmessage
        arm_and_takeoff()
    elif task in ["r. t. l.", "return to launch", "abort"]:
        interrupt = True
        globalmessage = "executing command RTL"
        print globalmessage
        vehicle.mode = VehicleMode("RTL")
        vehicle.flush()
    elif task == "land":
        globalmessage = "executing command land"
        print globalmessage
        vehicle.mode = VehicleMode("LAND")
        vehicle.flush()

def do_go(data):      #   {"name":"GoIntent","slots":{"Unit":{"name":"Unit","value":"meters"},"Direction":{"name":"Direction","value":"north"},"Distance":{"name":"Distance","value":"5"}}}
    direction = str(data["slots"]["Direction"]["value"])
    distance = int(data["slots"]["Distance"]["value"])
    unit = str(data["slots"]["Unit"]["value"])  # for now this is hard coded at meters.

    logger.info("GO " + direction + " " + str(distance) + " " + unit)

    # convert distance to meters
    if unit in ["feet", "foot"]:
        distance = 0.3048 * distance
    elif unit in ["yard", "yards"]:
        distance = 0.9144 * distance

    my_yaw = vehicle.attitude.yaw
    logger.debug("current bearing = " + str(my_yaw))
    # goto(dNorth, dEast, gotoFunction=vehicle.commands.goto):
    if direction == "north":
        goto(distance, 0)  # TODO:  this shouldn't be 0, it should be the current NED value for this axis
    elif direction == "south":
        distance = distance * -1  # convert for negative NED value
        goto(distance, 0)
    elif direction == "east":
        goto(0, distance)
    elif direction == "west":
        distance = distance * -1
        goto(0, distance)
    elif direction in ["forward", "forwards", "straight", "straight ahead"]:
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
    logger.debug("calc xy: distance = %s, angle= %s", distance, angle)
    if units != "radians":
        angle = math.radians(angle)
    angle = normalize_yaw(angle)
    x = distance * (math.cos(angle))
    y = distance * (math.sin(angle))
    return (round (x, 1) , round(y, 1))


def convert_yaw_to_degrees(angle):
    angle = normalize_yaw(angle)
    return math.degrees(angle)


def do_turn(data, degrees=90):
    direction = str(data["slots"]["Direction"]["value"])
    logger.info("before turn, current bearing is " + str(convert_yaw_to_degrees(vehicle.attitude.yaw)))
    logger.info("TURNING " + direction + "  BY " + str(degrees))
    rotation = 0
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


def arm_and_takeoff(aTargetAltitude=5):
    """
    Arm vehicle and fly to aTargetAltitude.
    """
    global globalmessage
    globalmessage = "Basic pre-arm checks"
    print globalmessage
    # Don't let the user try to fly while autopilot is booting
    if vehicle.mode.name == "INITIALISING":
        globalmessage = "Waiting for vehicle to initialise"
        print globalmessage
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        globalmessage = "Waiting for GPS...:", vehicle.gps_0.fix_type
        print globalmessage
        time.sleep(1)

    globalmessage = "Arming motors"
    print globalmessage
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.flush()

    while not vehicle.armed and not api.exit:
        globalmessage = " Waiting for arming..."
        print globalmessage
        time.sleep(1)

    globalmessage = "Taking off!"
    print globalmessage
    vehicle.commands.takeoff(aTargetAltitude)  # Take off to target altitude
    vehicle.flush()

    # Wait until the vehicle reaches a safe height before processing the goto
    # otherwise command after Vehicle.commands.takeoff will execute immediately.
    while not api.exit:
        globalmessage = " Altitude: ", vehicle.location.alt
        print globalmessage
        if vehicle.location.alt >= aTargetAltitude * 0.95:  # Just below target, in case of undershoot.
            globalmessage = "Reached target altitude"
            print globalmessage
            break;
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


def goto(dNorth, dEast, gotoFunction=vehicle.commands.goto):
    currentLocation = vehicle.location
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    vehicle.flush()

    global globalmessage
    while not api.exit and vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
        remainingDistance = get_distance_metres(vehicle.location, targetLocation)
        globalmessage = "Distance to target: ", remainingDistance
        print globalmessage
        if remainingDistance <= targetDistance * 0.1:  # Just below target, in case of undershoot.  WAS 0.01
            globalmessage = "Reached target"
            print globalmessage
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
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return Location(newlat, newlon, original_location.alt, original_location.is_relative)


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


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
#    print("Connected with result code " + str(rc))
    logger.info("connected with result code " + str(rc))
    # Subscribing in on_connect() means that if we lose the connection and      # reconnect then subscriptions will be renewed.    #    client.subscribe("$SYS/#")
    client.subscribe(topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    logger.info("msg received, payload = " + str(msg.payload))
    data = json.loads(str(msg.payload))
#    print "heading = " + str(math.degrees(data["Attitude"]["yaw"]))
#   figure out INTENT -- was it GO, COMMAND, TURN, etc  -- may add "FACE" at some time to point to Region of Interest (ROI)
    if None == data["name"]:
        return
    elif data["name"] == "CommandIntent":
        do_command(data)
    elif data["name"] == "GoIntent":
        do_go(data)
    elif data["name"] == "TurnIntent":
        do_turn(data)
def on_log(client, userdata, level, buf):
    logger.debug(buf)



def logger_init():
    logger.setLevel(logging.DEBUG)
    log_file_size = 1024 * 1024 * 1  # 1 MB
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(process)d - %(name)s : %(message)s')
    fh = logging.handlers.RotatingFileHandler('logs/echodronectl.log', maxBytes=log_file_size, backupCount=5)
    fh.setFormatter(formatter)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(formatter)
    logger.addHandler(fh)
    logger.addHandler(sh)
    logger.info('******************************************')
    logger.info('Starting up...')


logger_init()

logger.info('setting up mqtt client')
client = mqtt.Client(client_id="echodronectl.py")
logger.info('completed setting up mqtt client')
client.on_connect = on_connect
client.on_message = on_message
client.on_log = on_log
client.tls_set(root_cert,
               certfile = cert_file,
               keyfile = key_file,
               tls_version=ssl.PROTOCOL_TLSv1_2,
               ciphers=None)

client.connect(host, 8883, 60)

#client.loop_forever()

run = True
while run:
    client.loop()
    time.sleep(1)

    #vehicle = api.get_vehicles()[0]  # Get the connected vehicle (currently only one vehicle can be returned).

    try:
        mypayload = '''{
        "Location": {
            "lat": %s ,
            "lon": %s ,
            "alt": %s ,
            "is_relative": "%s"
        },
        "Attitude": {
            "pitch": %s,
            "yaw": %s,
            "roll": %s
        },
        "Velocity": %s,
        "GPSInfo": {
            "fix_type": %s,
            "satellites_visible": %s
            },
        "Groundspeed": %s,
        "Airspeed": %s,
        "Mount status": [%s, %s, %s],
        "Battery": {
            "voltage": %s,
            "current": %s,
            "level": %s
            },
        "Mode": "%s",
        "Armed": "%s",
        "StatusMessage": "%s"
        }''' % (vehicle.location.lat, vehicle.location.lon, vehicle.location.alt, vehicle.location.is_relative, vehicle.attitude.pitch, vehicle.attitude.yaw,
                vehicle.attitude.roll, vehicle.velocity, vehicle.gps_0.fix_type, vehicle.gps_0.satellites_visible, vehicle.groundspeed, vehicle.airspeed,
                str(vehicle.mount_status[0]).replace('None', '"None"'), str(vehicle.mount_status[1]).replace('None', '"None"'),
                str(vehicle.mount_status[2]).replace('None', '"None"'), vehicle.battery.voltage, str(vehicle.battery.current).replace('None', '"None"'),
                str(vehicle.battery.level).replace('None', '"None"'), vehicle.mode.name, vehicle.armed, globalmessage)

        # TODO:  should I clear globalmessage here?  wait till there is a new status?
        client.publish(topic, mypayload)

    except (TypeError):
        pass
