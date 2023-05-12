#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
from estop_nogui import EstopNoGui
from movement import Movement
#137.146.188.201

#create Spot object
#spot = Movement("user", "vd87k7o35nrs", "192.168.1.10")
spot = Movement("user", "vd87k7o35nrs", "137.146.188.201")


def messageDecoder(client, userdata, msg):
	data = msg.payload.decode(encoding='UTF-8')
	data.replace(" ","")
	
	if data == "on":
		print("on")
	elif data == "off":
		print("off")
		
	#receive data from client	        
	print ("Received Command: ", data)
	if data == "start" or data == "launch": 
		if spot.auth():
			print("authenticated")
		spot.toggle_power()
		print("powered on")
		spot.stand()
		print("standing")
	elif data == "shutdown":
		spot.shutdown()
	elif data == "forward" or data == "forwards":
		spot.move_forward()
	elif data == "backwards" or data == "backward":
		spot.move_backward()
	elif data == "sit" or data == "set":
		spot.sit()
	elif data == "stand":
		spot.stand()
	elif data == "left":
		spot.strafe_left()
	elif data == "right":
		spot.strafe_right()
	elif data == "turnleft":
		spot.turn_left()
	elif data == "turnright":
		spot.turn_right()
	elif data == "stop":
		spot.stop()
	elif data == "mouth":
		spot.toggle_mouth()
	elif data == "speedup":
		spot.increase_speed()
	elif data == "speeddown":
		spot.decrease_speed()
	elif data == "commandup":
		spot.increase_accuracy()
	elif data == "commanddown":
		spot.decrease_accuracy()
	elif "commandset" in data:
		spot.set_command(data[10:])
	elif data == "stow":
		spot.stow()
	elif data == "unstow":
		spot.unstow()
	elif data == "battRoll" or data == "rollover":
		spot.battery_change_pose()
	elif data == "fetch":
		spot.arm_grasp()
	elif data == "fetchtug":
		spot.fetch_tug()
	elif data == "off":
		spot.sit()
		spot.toggle_estop()
		spot.toggle_power()
		spot.toggle_lease()
	else:
		print("Received unrecognized data")
		print(data)


def connectionStatus(client, userdata, flags, rc):
	mqttClient.subscribe("rpi/mssg")
	mqttClient.subscribe("rpi/directControl")


clientName = "RPI"
#serverAddress = "137.146.255.24" # guest access
#serverAddress = "192.168.1.2" #irobot
serverAddress = "137.146.188.247"

mqttClient = mqtt.Client(clientName)

mqttClient.on_connect = connectionStatus
mqttClient.on_message = messageDecoder

mqttClient.connect(serverAddress)
mqttClient.loop_forever()



print ("Waiting for instructions...")



