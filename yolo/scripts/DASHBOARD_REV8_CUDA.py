###########################################################################################################################################################
#librerie utilizzate
###########################################################################################################################################################

import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import numpy as np
import time
import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image
import customtkinter
from ultralytics import YOLO
import torch
import io
import threading
import subprocess

###########################################################################################################################################################
#impostazioni della GUI
###########################################################################################################################################################

def stop_transmission():									#funzione che quando viene chiamata chiude tutto il programma
	client.disconnect()										#disconnessione dal client
	window.destroy()										#chiusura script

#def weight_transmission():									#funzione che quando viene chiamata manda i pesi aggiornati alla macchina
#	file_path = "best.pt"									#percorso locazione dei pesi aggiornati
#	with open(file_path, "rb") as file:						#impostazione per leggere (serializzando) i pesi
#		file_data = file.read()								#lettura serializzata
#	client.publish(topic_weight, file_data)					#pubblicazione dei pesi aggiornati sul topic
#	print("Weight published correctly.")

imminent_update = False										#boolean da portare a True quando c'è in programma un riavvio dello script
def weight_update():										#funzione che quando viene chiamata carica i nuovi pesi sul modello
	global imminent_update
	imminent_update = True
	time.sleep(0.5)											#500ms in cui nel thread di YOLO viene concluso il loop in cui ci si trova e NON se ne
															#	inizia uno nuovo
	client.disconnect()										#disconnessione dal client
	print("Weight updated correctly.")
	window.destroy()										#chiusura di tutte le finestre aperte dallo script
	subprocess.call("python3 DASHBOARD_REV8_CUDA.py", shell=True)		#viene richiamato lo script per avviarlo e riaprire le GUI con gli aggiornamenti caricati

exit_acquisition = True										#booleano da settare False quando si vogliono acquisire immagini da salvare in locale
def acquisition():											#loop che porta il boolean a False e inizia l'acquisizione di immagini
	global exit_acquisition
	exit_acquisition = False

def stop_acquisition():										#loop che porta il boolean a True e mette in pausa l'acquisizione di immagini
	global exit_acquisition
	exit_acquisition = True

def master_launch():
	client.publish(topic_master, "Launch the master.")

def ping_launch():
	client.publish(topic_ping, "ping...")

command_kill="rosnode kill /motors"
def kill_launch():
	subprocess.run(command_kill, shell=True, check=True)

customtkinter.set_appearance_mode("dark")					#stile del programma uguale a quello del sistema (dark/light)
customtkinter.set_default_color_theme("green")				#colore tema dei bottoni

window = customtkinter.CTk()								#inizializzazione widget principale della GUI (dashboard)
window.geometry("1280x720")									#impostazione dimensioni widget principale
window.title("Dashboard")									#titolo widget principale

window.columnconfigure(2, weight=1)							#setup numero colonne e ampiezza di ogni colonna
window.rowconfigure(1, weight=1)							#setup numero righe e ampiezza di ogni riga

telemetry_box = Frame(window, bg="#242424")					#creazione del frame 'telemetry_box', utilizzato in questa GUI per visualizzare la telemetria
telemetry_box.grid(row=1, column=1)							#posizionamnto del frame nella GUI
telemetry_box.columnconfigure(1, weight=1)					#numero di colonne che ha il frame
telemetry_box.rowconfigure(0, weight=1)						#numero di righe che ha il frame

detection_box = Frame(window, bg="#242424")					#creazione frame per visualizzare le rilevazione dei cartelli
detection_box.grid(row=0, column=2, padx=10)
detection_box.columnconfigure(0, weight=1)
detection_box.rowconfigure(3, weight=1)

buttons_box = Frame(window, bg="#242424")					#creazione frame per visualizzare i bottoni
buttons_box.grid(row=0, column=0, padx=20)
buttons_box.columnconfigure(0, weight=1)
buttons_box.rowconfigure(7, weight=1)

stop_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Stop transmission", command=stop_transmission, fg_color="red")		#creazione bottone per terminare lo script con assegnazione nome e
															#	funzione da richiamare una volta premuto
stop_button.grid(row=0, column=0, pady=50)					#posizionamento bottone per terminare lo script

#weight_transmission_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
#text="Weights transmission", command=weight_transmission)	#bottone per trasmissione pesi aggiornati
#weight_transmission_button.grid(row=1, column=0, pady=10)

weight_update_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Weights update", command=weight_update)				#bottone per riavvio dello script con aggiornamenti caricati
weight_update_button.grid(row=2, column=0, pady=10)

acquisition_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Start acquisition", command=acquisition)				#bottone per iniziare acquisizione immagini
acquisition_button.grid(row=3, column=0, pady=10)

stop_acquisition_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Stop acquisition", command=stop_acquisition)			#bottone per mettere in pausa l'acquisizione immagini
stop_acquisition_button.grid(row=4, column=0, pady=10)

master_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Master.launch", command=master_launch, fg_color="darkgreen")			#bottone per mettere in pausa l'acquisizione immagini
master_button.grid(row=5, column=0, pady=10)

kill_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Kill master", command=kill_launch, fg_color="darkgreen")				#bottone per mettere in pausa l'acquisizione immagini
kill_button.grid(row=6, column=0, pady=10)

ping_button = customtkinter.CTkButton(master=buttons_box, width=200, height=32, font=("helvetica", 18),
text="Ping check", command=ping_launch, fg_color="darkgreen")				#bottone per mettere in pausa l'acquisizione immagini
ping_button.grid(row=7, column=0, pady=10)

label_video = Label(window)
label_video.grid(row=0, column=1, pady=40)

###########################################################################################################################################################
#impostazioni della connessione col broker
###########################################################################################################################################################

def on_connect(client, userdata, flags, rc):				#funzione che viene chiamata quando avviene la connessione, se voglio che
		if rc==0:											#	succedano cose quando mi connetto, metto tutto dentro questa funzione
			print("Connected OK")
		else:
			print("Connection error")
		client.subscribe([(topic_stream, 0),(topic_temp, 0),(topic_voltage, 0), (topic_ping, 0)])			#sincronizzazione con topic
		print("Subscribed to: "+topic_stream+", "+topic_temp+", "+topic_voltage+", "+topic_ping)

contrast_factor = 1.5
brightness_factor = 30
kernel = np.array([[0, -1, 0], [-1, 5,-1], [0, -1, 0]])

def on_message(client, userdata, msg):						#funzione chiamata quando si riceve un messaggio (subscriber) se voglio che
															#	succedano cose quando ricevo messaggi, metto tutto dentro questa funzione
	global frame
	global contrast_factor
	global brightness_factor
	global kernel
	global frame_sharp
	if msg.topic == topic_stream:
		img = base64.b64decode(msg.payload)					#conversione della stringa ricevuta in immagine
		npimg = np.frombuffer(img, dtype=np.uint8)
		frame = cv.imdecode(npimg, 1)
		frame_corrected = cv.convertScaleAbs(frame, alpha=contrast_factor, beta=brightness_factor)	#correzione contrasto e luminosità
		frame_sharp = cv.filter2D(src=frame_corrected,ddepth=-1,kernel=kernel)	#rimozione effetto blur (sfocatura)
	if msg.topic == topic_temp:								#ricezione del topic di temperatura del raspberry
		text_temp = msg.payload.decode("utf-8")				#decodifica da bytes a stringa
		label_temp.config(text="Temperature: "+text_temp+"°C")		#funzione per aggiornare il label senza doverlo riposizionare
	if msg.topic == topic_voltage:							#ricezione del topic di tensione delle batterie
		text_voltage = msg.payload.decode("utf-8")
		label_voltage.config(text="Voltage: " +text_voltage+"V")
	if msg.topic == topic_ping:
		ping_text = msg.payload.decode("utf-8")
		if ping_text == "pong!":
			print(ping_text)

text_temp = "42"
text_voltage = "7.8"
label_temp = Label(telemetry_box, text="Temperature: "+text_temp+"°C", fg="white", font=("helvetica", 20), bg="#242424")
label_temp.grid(row=0, column=0, padx=10)					#creazione e posizionamento label di temperatura
label_voltage = Label(telemetry_box, text="Voltage: " +text_voltage+"V", fg="white", font=("helvetica", 20), bg="#242424")
label_voltage.grid(row=0, column=1, padx=10)				#creazione e posizionamento label di tensione

frame = np.zeros((240, 320, 3), np.uint8)					#valore generico iniziale alla variabile che conterrà i frame185
frame_sharp = np.zeros((240, 320, 3), np.uint8)
broker = "10.42.0.1"										#indirizzo IPv4 (cambia se cambio connessione) del PC dove ho il broker
topic_stream = "/video/streaming"							#topic sul quale pubblicare messaggi
topic_temp = "/data/temperature"
topic_voltage = "/data/voltage"
topic_weight = "/data/weight"
topic_ping = "/ping"
topic_master = "/master"
topic_kill = "/kill"
port = 1883													#porta predefinita MQTT, posso anche non specificarla (se è predefinita)
client_id = "dashboard_cuda1440"							#è l'ID che distingue il client, non devono girare contemporaneamente più client con lo stesso ID

###########################################################################################################################################################
#calcolo distanza cartello identificato
###########################################################################################################################################################

def signal_name_association(classification):				#funzione che riceve in input il numero identificativo del cartello e dà come output
															#	il nome del corrispondente cartello
	if classification == 0:
		signal_name = "Attraversamento Pedonale"
	elif classification == 1:
		signal_name = "Dare Precedenza"
	elif classification == 2:
		signal_name = "Divieto di Accesso"
	elif classification == 3:
		signal_name = "Divieto di Sosta"
	elif classification == 4:
		signal_name = "Segnale Stradale"
	elif classification == 5:
		signal_name = "Stop"
	return signal_name										#return di una stringa contenente il nome del segnale associato al numero di classificazione
															#	ricevuto in input

def height_to_distance(height):								#funzione che converte il valore di pixel dell'altezza in una distanza corrispondente stimata
	if height <= 31:
		distance = 1.6
	elif 31 < height <= 35:
		distance = 1.5
	elif 35 < height <= 37:
		distance = 1.4
	elif 37 < height <= 39:
		distance = 1.3
	elif 39 < height <= 42:
		distance = 1.2
	elif 42 < height <= 46:
		distance = 1.1
	elif 46 < height <= 50:
		distance = 1
	elif 50 < height <= 55:
		distance = 0.9
	elif 55 < height <= 65:
		distance = 0.8
	elif 65 < height <= 79:
		distance = 0.7
	elif 79 < height <= 90:
		distance = 0.6
	elif 90 < height <= 107:
		distance = 0.5
	elif 107 < height <= 131:
		distance = 0.4
	elif 131 < height <= 155:
		distance = 0.3
	elif 155 < height:
		distance = 0.2
	return distance

results = []												#inizializzazione del vettore contenente le altezze delle boxes
def detected_distance_estimation():
	global detected_class									#vettore di dimensione dinamica che indica le coordinate delle boxes delle detection
	global results											#variabile vettore in cui salvo le altezze delle boxes delle detection valide
	if len(detected_class) == 1:								#caso in cui è stato rilevato un solo cartello
		height1 = detected_class[0][2] - detected_class[0][0]	#calcolo altezza detection
		width1 = detected_class[0][3] - detected_class[0][1]	#calcolo larghezza detection
		if (width1-15) <= height1 <= (width1+15):				#controllo se la detection è valida o meno (se la larghezza è molto diversa dall'altezza
																#	probabilmente è una rilevazione errata e non la considero nella dashboard)
			results[0] = height_to_distance(width1)			#salvo il valore di altezza solo se la rilevazione è valida
		results[1] = 0										#gli altri valori li tengo a zero, così la memoria delle precedenti rilevazioni è sempre pulita
		results[2] = 0
		results[3] = 0
	elif len(detected_class) == 2:							#caso in cui sono stati rilevati due cartelli
		height1 = detected_class[0][2] - detected_class[0][0]
		width1 = detected_class[0][3] - detected_class[0][1]
		if (width1-15) <= height1 <= (width1+15):
			results[0] = height_to_distance(width1)
		height2 = detected_class[1][2] - detected_class[1][0]
		width2 = detected_class[1][3] - detected_class[1][1]
		if (width2-15) <= height2 <= (width2+15):
			results[1] = height_to_distance(width2)
		results[2] = 0
		results[3] = 0
	elif len(detected_class) == 3:							#caso in cui sono stati rilevati tre cartelli
		height1 = detected_class[0][2] - detected_class[0][0]
		width1 = detected_class[0][3] - detected_class[0][1]
		if (width1-15) <= height1 <= (width1+15):
			results[0] = height_to_distance(width1)
		height2 = detected_class[1][2] - detected_class[1][0]
		width2 = detected_class[1][3] - detected_class[1][1]
		if (width2-15) <= height2 <= (width2+15):
			results[1] = height_to_distance(width2)
		height3 = detected_class[2][2] - detected_class[2][0]
		width3 = detected_class[2][3] - detected_class[2][1]
		if (width3-15) <= height3 <= (width3+15):
			results[2] = height_to_distance(width3)
		results[3] = 0
	elif len(detected_class) == 4:							#caso in cui sono stati rilevati quattro cartelli
		height1 = detected_class[0][2] - detected_class[0][0]
		width1 = detected_class[0][3] - detected_class[0][1]
		if (width1-15) <= height1 <= (width1+15):
			results[0] = height_to_distance(width1)
		height2 = detected_class[1][2] - detected_class[1][0]
		width2 = detected_class[1][3] - detected_class[1][1]
		if (width2-15) <= height2 <= (width2+15):
			results[1] = height_to_distance(width2)
		height3 = detected_class[2][2] - detected_class[2][0]
		width3 = detected_class[2][3] - detected_class[2][1]
		if (width3-15) <= height3 <= (width3+15):
			results[2] = height_to_distance(width3)
		height4 = detected_class[3][2] - detected_class[3][0]
		width4 = detected_class[3][3] - detected_class[3][1]
		if (width4-15) <= height4 <= (width4+15):
			results[3] = height_to_distance(width4)
	elif len(detected_class) == 0:
		results = [0,0,0,0]
	return results											#la funziona ritorna il vettore con le altezze [in pixels] dei primi 4 cartelli rilevati real-time

###########################################################################################################################################################
#setup di YOLO
###########################################################################################################################################################
detected_class = []											#inizializzazione del vettore in cui vengono salvati i vettori Tensor (output di YOLO) che
															#	inidicano le coordinate delle boxes delle detection in real-time (è un vettore dinamico)
acquisition_counter = 1										#contatore utilizzato per l'acquisizione di immagini da salvare in locale
detection_counter = 1										#contatore utilizzato per rendere più stabile la grafica sulla GUI delle distanze dei segnali
def YOLO_loop():											#loop per il funzionamento di YOLO
	global imminent_update									#boolean per il riavvio dello script
	global frame											#frame ricevuto e caricato dal thread in cui gira MQTT
	global acquisition_counter
	global detected_class
	global detection_counter
	global frame_sharp
	while True:
		if imminent_update == False:						#quando non è previsto un riavvio dello script, eseguo tutto il loop di YOLO
			results = model.predict(frame_sharp, stream=True, verbose=False, imgsz=(1440, 1088), device=0, iou=0.5, conf=0.35)		#carica in una variabile tutti gli output di YOLO
			for r in results:
				Annotated_frame = r.plot()					#prende l'output di YOLO e lo carica come immagine sulla memoria RAM in una variabile 'annotated_frame'
				annotated_frame = cv.resize(Annotated_frame, (960, 720))
				detected_class = r.boxes.xyxy.cpu().numpy()		#legge i valori delle coordinate delle boxes e li salva in un vettore dinamico. '.numpy()' serve
															#	a convertire il vettore Tensor in un normale vettore da poter utilizzare normalmente
				detected_class_name = r.boxes.cls.cpu().numpy()	#salva il numero corrispondente alla classe del cartello rilevato
		#posizionamento video streaming#
			Frame = cv.cvtColor(annotated_frame, cv.COLOR_BGR2RGBA)
			IMM = Image.fromarray(Frame)
			imgtk = ImageTk.PhotoImage(image=IMM)
			label_video.configure(image=imgtk)
			label_video.imgtk = imgtk
			if(exit_acquisition == False):					#loop che viene eseguito quando si vogliono acquisire immagini da salvare in locale
				print(acquisition_counter)
				acquisition_counter = acquisition_counter+1
				if(acquisition_counter%20 == 0):			#il '%60' va modificato per decidere il rate con cui salvare le immagini, con '%60' vuol dire che
															#	ogni 60 frame salvo un frame nella memoria locale (salvataggio al percorso nella riga sottostante)
					acquisition_path = r'acquisition_'+str(acquisition_counter)+'.jpg'
					cv.imwrite(acquisition_path, frame)
		#posizionamento dati cartelli rilevati#
			distances = detected_distance_estimation()		#il vettore results della funzione chiamata, viene salvato nel vettore 'distances'
			if distances[0] != 0:
				signal_1 = signal_name_association(detected_class_name[0])
				text_signal_1 = str(signal_1) + " - Distance: " + str(distances[0]) + "m"
			else:
				text_signal_1 = ''
			if distances[1] != 0:
				signal_2 = signal_name_association(detected_class_name[1])
				text_signal_2 = str(signal_2) + " - Distance: " + str(distances[1]) + "m"
			else:
				text_signal_2 = ''
			if distances[2] != 0:
				signal_3 = signal_name_association(detected_class_name[2])
				text_signal_3 = str(signal_3) + " - Distance: " + str(distances[2]) + "m"
			else:
				text_signal_3 = ''
			if distances[3] != 0:
				signal_4 = signal_name_association(detected_class_name[3])
				text_signal_4 = str(signal_4) + " - Distance: " + str(distances[3]) + "m"
			else:
				text_signal_4 = ''
			detection_counter = detection_counter + 1
			if (detection_counter%6 == 0):					#una sorte di filtro passa basso che rende l'output sulla GUI più robusto contro
				label_signal_1.config(text=text_signal_1)	#	i pochi frame in cui si perde la detection del cartello o si detecta
				label_signal_2.config(text=text_signal_2)	#	un cartello lontano che sparisce subito. La GUI aggiorna l'output ogni 6 frame
				label_signal_3.config(text=text_signal_3)
				label_signal_4.config(text=text_signal_4)
				detection_counter = 1
	pass
text_signal_1 = ''
text_signal_2 = ''
text_signal_3 = ''
text_signal_4 = ''
label_signal_1 = Label(detection_box, text=text_signal_1, fg="white", font=("helvetica", 20), bg="#242424")
label_signal_1.grid(row=0, column=0, pady=10, padx=10)
label_signal_2 = Label(detection_box, text=text_signal_2, fg="white", font=("helvetica", 20), bg="#242424")
label_signal_2.grid(row=1, column=0, pady=10, padx=10)
label_signal_3 = Label(detection_box, text=text_signal_3, fg="white", font=("helvetica", 20), bg="#242424")
label_signal_3.grid(row=2, column=0, pady=10, padx=10)
label_signal_4 = Label(detection_box, text=text_signal_4, fg="white", font=("helvetica", 20), bg="#242424")
label_signal_4.grid(row=3, column=0, pady=10, padx=10)

model = YOLO('signals_detection.pt')						#caricamento di un modello pre-trained
print("YOLO model loaded: OK")
YOLO_thread = threading.Thread(target=YOLO_loop)			#creazione di un thread secondario su cui far girare YOLO in parallelo ad MQTT e GUI. In questo
															#	modo YOLO e streaming video possono funzionare a frequenze differenti senza che uno faccia
															#	da bottleneck all'altro
YOLO_thread.start()											#start del thread di YOLO

###########################################################################################################################################################
#client-broker connection
###########################################################################################################################################################

client = mqtt.Client(client_id)								#associa l'ID al client, se ometto l'argomento della funzione, mi genera un id casuale
client.on_connect = on_connect								#chiamata funzione che resta in attesa finchè non ci si connette
client.on_message = on_message								#chiamata della funzione che dà output ogni volta che ricevo un messaggio su un topic
client.username_pw_set("Pollicino", "Mandarino11")			#setting credenziali di accesso broker (se sono state settate sul broker)
															#	[USERNAME/PASSWORD]
print("Connecting to broker: ", broker)
client.connect(broker, port)								#connessione al broker, usando le credenziali impostate nella funzione precedente

###########################################################################################################################################################
#MQTT loop starting
###########################################################################################################################################################

client.loop_start()											#inizio del loop di MQTT

window.mainloop()											#loop GUI

client.loop_stop()											#chiusura del loop di MQTT e quindi dello script
