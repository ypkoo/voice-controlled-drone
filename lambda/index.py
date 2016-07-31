"""
By YP KOO

Lambda function for Voice controlled drone.
Ported to Python based on Chris Synan's code in node.js.
"""

import json
import paho.mqtt.client as mqtt
import ssl, sys, time
from conf import *

pub_success = False
timeout = 3

def on_connect(mqttc, obj, flags, rc):
	if rc==0:
		print ("Subscriber Connection status code: "+str(rc)+" | Connection status: successful")
	elif rc==1:
		print ("Subscriber Connection status code: "+str(rc)+" | Connection status: Connection refused")

def on_publish(mqttc, userdata, mid):
	global pub_success
	pub_success = True

def lambda_handler(event, context):
	try:
		if event["session"]["application"]["applicationId"] != app_id:
			return "Invalid Application ID"
			

		mqttc = mqtt.Client(client_id="aws-client")
		mqttc.on_connect = on_connect
		mqttc.on_publish = on_publish

		mqttc.tls_set(root_cert, 
				certfile=cert_file, 
				keyfile=key_file, 
				cert_reqs=ssl.CERT_REQUIRED, 
				tls_version=ssl.PROTOCOL_SSLv23, 
				ciphers=None)

		mqttc.connect(host, port)

		mqttc.loop_start()

		ret = None
		is_launch = False

		request = event["request"]
		session = event["session"]
		request_type = event["request"]["type"]

		#print("Received event: " + json.dumps(event, indent=2))

		if event["session"]["new"]:
			on_session_started(request, session)

		if request_type == "LaunchRequest":
			ret = on_launch(request, session)
			global pub_success
			pub_success = True
			is_launch = True
		elif request_type == "IntentRequest":
			ret = on_intent(request, session, mqttc)
		elif request_type == "SessionEndedRequest":
			ret = on_session_ended(request, session, mqttc)
			global pub_success
			pub_success = True

		while not pub_success:
			time.sleep(0.5)

		if not is_launch:
			time.sleep(timeout)
			
		mqttc.disconnect()

		return ret

	except Exception, e:
		print "EXCEPTION in handler:  " + str(e)
		return "EXCEPTION in handler:  " + str(e)

	


def on_session_started(request, session):
	print "new session started!"

def on_launch(request, session):
	return welcome_response()

def on_intent(intent_request, session, mqtt_client):
	intent = intent_request["intent"]
	intent_name = intent_request["intent"]["name"]

	if intent_name == "Go":
		return do_go_intent(intent, session, mqtt_client)
	elif intent_name == "Command":
		return do_command_intent(intent, session, mqtt_client)
	elif intent_name == "Turn":
		return do_turn_intent(intent, session, mqtt_client)

def on_session_ended(request, session, mqtt_client):
	# mqtt_client.disconnect()
	pass

def do_go_intent(intent, session, mqtt_client):

	direction = intent['slots']['Direction']['value']
	distance = intent['slots']['Distance']['value']
	unit = intent['slots']['Unit']['value']

	card_title = "Drone Go"
	reprompt_text = ""
	session_attributes = {}
	should_end_session = False
	speech_output = "Going " + direction + " " + distance + " " + unit

	msg_to_pub = json.dumps(intent)

	mqtt_client.publish(topic, msg_to_pub, qos=1)

	return build_response(session_attributes, build_speechlet_response(card_title, speech_output, reprompt_text, should_end_session))

def do_command_intent(intent, session, mqtt_client):

	task = intent['slots']['Task']['value']

	card_title = "Executing Drone command " + task
	reprompt_text = ""
	session_attributes = {}
	should_end_session = False
	speech_output = "Executing command " + task

	msg_to_pub = json.dumps(intent)

	mqtt_client.publish(topic, msg_to_pub, qos=1)

	return build_response(session_attributes, build_speechlet_response(card_title, speech_output, reprompt_text, should_end_session))

def do_turn_intent(intent, session, mqtt_client):
	
	direction = intent['slots']['Direction']['value']

	card_title = "Drone turning " + direction
	reprompt_text = ""
	session_attributes = {}
	should_end_session = False
	speech_output = "Turning " + direction

	msg_to_pub = json.dumps(intent)

	mqtt_client.publish(topic, msg_to_pub, qos=1)

	return build_response(session_attributes, build_speechlet_response(card_title, speech_output, reprompt_text, should_end_session))

def welcome_response():
	session_attributes = {}
	card_title = "Welcome"
	speech_output = "Welcome to the drone control."

	reprompt_text = "Drone ready for command."
	should_end_session = False

	return build_response(session_attributes, build_speechlet_response(card_title, speech_output, reprompt_text, should_end_session))

def build_speechlet_response(title, output, reprompt_text, should_end_session):

	return {
		'outputSpeech': {
			'type': "PlainText",
			'text': output
		},
		'card': {
			'type': "Simple",
			'title': title,
			'content': output
		},
		'reprompt': {
			'outputSpeech': {
				'type': "PlainText",
				'text': reprompt_text
			}
		},
		'shouldEndSession': should_end_session
	}

def build_response(session_attributes, speechlet_response):

	return {
		'version': "1.0",
		'sessionAttributes': session_attributes,
		'response': speechlet_response
	}

