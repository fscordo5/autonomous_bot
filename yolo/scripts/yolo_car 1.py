#!/usr/bin/env python3
###########################################################################################################################################################
#client object [il client è tutto lo script, due script diversi sono due client diversi]
###########################################################################################################################################################

import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import rospy
import subprocess
import io

def on_connect(client, userdata, flags, rc):		#funzione che viene chiamata quando avviene la connessione, se voglio che
		if rc==0:									#succedano cose quando mi connetto, metto tutto dentro questa funzione
			print("Connected: OK")
		else:
			print("Connected: error")

rospy.init_node("yolo_node")

broker = "192.168.2.185"							#indirizzo IPv4 (cambia se cambio connessione) del PC dove ho il broker
topic_stream = "/video/streaming"					#topic sul quale pubblicare messaggi
topic_temp = "/data/temperature"
topic_voltage = "/data/voltage"
port = 1883											#porta predefinita MQTT, posso anche non specificarla (se è predefinita)
client_id = "car"									#è l'ID che distingue il client, non devono girare contemporaneamente più client con lo stesso ID
													#(ANCHE SE SONO SU PC DIFFERENTI)
cap = cv.VideoCapture(0)							#nella variabile cap metto l'acquisizione della webcam, il valore nella funzione indica il numero di webcam
													#che voglio usare, se metto 0 uso l'unica webcam che ho (se ne ho una sola)

###########################################################################################################################################################
#client-broker connection
###########################################################################################################################################################

client = mqtt.Client(client_id)						#associa l'ID al client, se ometto l'argomento della funzione, mi genera un id casuale
client.on_connect=on_connect						#chiamata funzione che resta in attesa finchè non ci si connette
client.username_pw_set("Pollicino", "Mandarino11")	#setting credenziali di accesso broker(se sono state settate sul broker)
print("Connecting to broker: ", broker)
try:
	client.connect(broker, port)					#connessione al broker, usando le credenziali impostate nella funzione precedente
except:
	rospy.logerr("CONNECTION FAILED, yolo will shutdown")
	exit()

###########################################################################################################################################################
#messages publishing on topic
###########################################################################################################################################################

client.loop_start()									#inizio del loop
print("Video streaming started.")
Temp = 42123
loop_counter = 1
rate = rospy.Rate(1)
try:
	while not rospy.is_shutdown():										#while che funziona finchè non premo un tasto (quando metto solo i ':'
													#significa che il while va all'infinito)
		ret, frame = cap.read()
		frame = cv.resize(frame, (1440, 1088))
		ret, buffer = cv.imencode('.jpg', frame)	#rendere la lettura del frame un figura .jpg
		encode_frame = base64.b64encode(buffer)		#utilizzo di base64 per convertire il frame .jpg in una stringa
		client.publish(topic_stream, encode_frame)	#pubblicazione del messaggio sul topic						#limitazione FPS per non intasare il subscriber (YOLO sul subscriber è molto lento)
		#if loop_counter%30 == 0:
		#	Temp = subprocess.check_output(['vcgencmd', 'measure_temp']).decode('utf-8')
		#	temperature = float(Temp.split('='[1].split('\'')[0])
		#	client.publish(topic_temp, temperature)
		rate.sleep()
except:
	pass
cap.release()
client.loop_stop()
client.disconnect()
