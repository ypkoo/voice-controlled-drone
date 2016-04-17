import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import ssl, sys


def on_connect(mqttc, obj, flags, rc):
    if rc==0:
        print ("Subscriber Connection status code: "+str(rc)+" | Connection status: successful")
    elif rc==1:
        print ("Subscriber Connection status code: "+str(rc)+" | Connection status: Connection refused")

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos)+"data"+str(obj))

def on_message(mqttc, obj, msg):
    print("Received message from topic: "+msg.topic+" | QoS: "+str(msg.qos)+" | Data Received: "+str(msg.payload))

mqttc = mqtt.Client(client_id="mqtt-test")
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_message = on_message

cert_path = "awsCerts/"
host = "A30ITWQ5LJOT4V.iot.us-east-1.amazonaws.com"
port = 8883
topic = "aws/things/GroundStation01/"
root_cert = cert_path + "root-CA.crt"
cert_file = cert_path + "3480a0ba5b-certificate.pem.crt"
key_file = cert_path + "3480a0ba5b-private.pem.key"

mqttc.tls_set(root_cert, 
        certfile=cert_file, 
        keyfile=key_file, 
        cert_reqs=ssl.CERT_REQUIRED, 
        tls_version=ssl.PROTOCOL_SSLv23, 
        ciphers=None)

mqttc.connect(host, port)
ret = mqttc.subscribe(topic, qos=1)
print ret
mqttc.publish(topic, "this is gcs.")

mqttc.loop_forever()
