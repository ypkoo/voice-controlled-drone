app_id = "amzn1.echo-sdk-ams.app.your-app-id"

cert_path = "awsCerts/"
host = "your-host.iot.us-east-1.amazonaws.com" # your REST API endpoint of AWS IoT
port = 8883
topic = "aws/things/your-thing-name/" # your MQTT topic of AWS IoT
root_cert = cert_path + "root-CA.crt"
cert_file = cert_path + "your-certificate.pem.crt"
key_file = cert_path + "your-private.pem.key"