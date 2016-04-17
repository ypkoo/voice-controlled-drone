import paho.mqtt.client as mqtt
import ssl, sys

cert_path = "awsCerts/"
host = "A30ITWQ5LJOT4V.iot.us-east-1.amazonaws.com"
port = 8883
topic = "aws/things/GroundStation01/"
root_cert = cert_path + "root-CA.crt"
cert_file = cert_path + "3480a0ba5b-certificate.pem.crt"
key_file = cert_path + "3480a0ba5b-private.pem.key"

def on_connect(mqttc, obj, flags, rc):
	if rc==0:
		print ("Subscriber Connection status code: "+str(rc)+" | Connection status: successful")
	elif rc==1:
		print ("Subscriber Connection status code: "+str(rc)+" | Connection status: Connection refused")

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.tls_set(root_cert, 
		certfile=cert_file, 
		keyfile=key_file, 
		cert_reqs=ssl.CERT_REQUIRED, 
		tls_version=ssl.PROTOCOL_SSLv23, 
		ciphers=None)






