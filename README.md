# voice-controlled-drone


This instruction explains how to control a drone with your voice. [Watch a demo video](https://www.youtube.com/watch?v=CEPlKKUS94I)

_Note: This work is based on Chris Synan's previous work, with few things modified. You can see his work [here](https://www.hackster.io/veggiebenz/voice-controlled-drone-with-amazon-echo-and-3dr-iris-c9fd2a)._

## Overall structure
![](https://github.com/ypkoo/voice-controlled-drone/blob/master/wiki/images/overall_structure.PNG)

## What you need
* Amazon Echo
* 3DR Solo
* A laptop (or a Raspberry Pi)

## Amazon IoT setting

1. Go to https://console.aws.amazon.com/iot/
2. Click 'Create a Resource' and click 'Create a thing'. Enter your thing's name and click 'Create'.

 _Note your REST API endpoint and MQTT topic at the right sidebar. You will need this later._
 ![](https://github.com/ypkoo/voice-controlled-drone/blob/master/wiki/images/thing_detail.png)
3. Click 'Create a certificate'. Check 'Activate' checkbox, and click '1-Click certificate create'. Download the public key, private key, and certificate.
4. Click the certificate that you just created, and click 'Actions' dropdown menu and click 'Attach a thing'.
5. Enter your thing's name and click 'Attach'.
6. Click 'Create a Resource' and click 'Create a policy'. Enter your policy name. Fill it up like below and click 'Add statement', and click 'Create'.

 ![](https://github.com/ypkoo/voice-controlled-drone/blob/master/wiki/images/create_policy.png)

## Lambda function setting

1. Go to https://console.aws.amazon.com/lambda/
2. Click 'Create a Lambda function', and click 'Next'.
3. Select 'AWS IoT' as your trigger. Check 'Enable trigger' checkbox and click 'Next'.

 ![](https://github.com/ypkoo/voice-controlled-drone/blob/master/wiki/images/configure_triggers.png)

4. Enter your lambda function name. Choose Python 2.7 as your runtime.
5. Set your role as below and click 'Next'. Click 'Create function'.

 ![](https://github.com/ypkoo/voice-controlled-drone/blob/master/wiki/images/lambda_setting.png)

 _Note your ARN at the right upper corner. You will need this later._

## Alexa skills kit setting

1. Go to https://developer.amazon.com/
2. Click 'Alexa' menu, and click 'Create an Alexa skill now' at right sidebar. Click 'Add a new skill'.
3. Fill up Alexa skill information as you want. Click 'Next'. 
 ![](https://github.com/ypkoo/voice-controlled-drone/blob/master/wiki/images/alexa_info.png)
 _Note your application ID. You will need this later._
4. Copy the intent schema and sample utterances below and paste into corresponding text fields. Click 'Next'. 
     
 **Intent Schema**  
 ```  
 {
   "intents":[
      {
         "intent":"Go",
         "slots":[
            {
               "name":"Direction",
               "type":"LITERAL"
            },
            {
               "name":"Distance",
               "type":"NUMBER"
            },
            {
               "name":"Unit",
               "type":"LITERAL"
            }
         ]
      },
      {
         "intent":"Command",
         "slots":[
            {
               "name":"Task",
               "type":"LITERAL"
            }
         ]
      },
      {
         "intent":"Turn",
         "slots":[
            {
               "name":"Direction",
               "type":"LITERAL"
            },
            {
               "name":"Rotation",
               "type":"NUMBER"
            }
         ]
      }
   ]
}
 ```  
    
 **Sample Utterances**  
 ```  
 Go go {forward | Direction} {ten | Distance} {feet | Unit}
 Go go {backward | Direction} {three | Distance} {yards | Unit}
 Go go {up | Direction} {twenty five | Distance} {meters | Unit}
 Go go {down | Direction} {one | Distance} {foot | Unit}
 Go go {left | Direction} {fifteen | Distance} {yard | Unit}
 Go go {right | Direction} {two | Distance} {meter | Unit}
 Go go {back | Direction} {four | Distance} {feet | Unit}
 Go go {straight | Direction} {five | Distance} {feet | Unit}
 Command command {Launch | Task}
 Command command {Land | Task}
 Command command {Return to Launch | Task}
 Command command {Stay | Task}
 Command command {Hold | Task}
 Turn turn {right|Direction} {ninety | Rotation} degrees
 Turn turn {left | Direction} {forty five | Rotation} degrees
 Turn turn {right|Direction}
 Turn turn {left|Direction}
 ```   

5. Select 'Lambda ARN' as your endpoint. Enter your ARN of your lambda function that you just created and click 'Next'.

## GCS setting / Lambda function update

1. Clone ``https://github.com/ypkoo/voice-controlled-drone.git`` to your laptop (or Raspberry Pi).
1. Run the following commands (using [virtualenv](https://virtualenvwrapper.readthedocs.io/en/latest/) is recommanded).

 ```
 sudo apt-get install python-pip python-dev
 pip install dronekit
 pip install paho-mqtt
 ```
1. Place your certificate files at gcs/awsCerts and lambda/awsCerts.
1. Modify gcs/conf.py and lambda/conf.py. They are self-explanatory.
1. Now we need to upload lambda function.
 * Go to http://docs.aws.amazon.com/cli/latest/userguide/cli-chap-getting-set-up.html and follow its instructions to set up your aws cli environment.
 * Modify your lambda/upload.sh file. It's self-explanatory.
 * Run ``./upload.sh``

## Solo setting

1. Go to https://dev.3dr.com/
1. Follow its instructions so you are ready to run gcs/gcs.py.

 _You may need 2 WiFi interfaces, 1 for accessing Solo and 1 for accessing the internet. Or you can access internet using ``solo wifi`` command. Read "solo Command Line Tool" section from the above url._

## Fly

1. Get your Solo and laptop ready.
1. cd to gcs/ directory, and run ``./gcs.py --connect solo``.
1. Enjoy!
