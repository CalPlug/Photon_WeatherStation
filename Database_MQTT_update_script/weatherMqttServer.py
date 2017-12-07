from pymongo import MongoClient
import paho.mqtt.client as mqtt
import re
import datetime

prev_msg = ''
data_row = []
#Improve readability of time

def parse_msg(msg) -> tuple:
	cleanResult = []
	for i in msg:
		if i == '*':
			cleanResult.append('')
		else:
			cleanResult.append(i.decode("ascii"))
	#Uncomment to test
	#print(cleanResult)
	return tuple(cleanResult)

def on_connect(client, userdata, flags, rc):
	print("Connected with result code " + str(rc))

#Add on message callbacks for all topics and construct tuple

def on_message(client, userdata, msg):

	global prev_msg
	global data_row


	demand_msg = str(msg.payload.decode("utf-8"))
	#Uncomment to test
	#print("Incoming msg payload is : " + demand_msg)
	data_row.append(msg.payload)
	if demand_msg == prev_msg:
		return
	else:
		prev_msg = demand_msg

	if(len(data_row) >= 14):

		tuple_msg = parse_msg(data_row)
	
		client = MongoClient('localhost', 27017)
		db = client.Weather_Station
		test_readings = db.Weather_Station_Test
		test_readings_data = {
		'utcTime':		  	  datetime.datetime.utcnow(),
		'Temperature_F':				tuple_msg[0],
		'Temperature_C':	 			tuple_msg[1],
		'Humidity_RH': 	 				tuple_msg[2],
		'Pressure_KPa': 				tuple_msg[3],
		'DewPoint_C': 					tuple_msg[4],
		'DewPoint_F': 					tuple_msg[5],
		'H2OPartialPressure_KPa':			tuple_msg[6],	 
		'WindSpeed_MPH': 				tuple_msg[7],		
		'WindDirection_DEG':				tuple_msg[8],
		'LightUV_INDEX':				tuple_msg[9],
		'LightVIS_ADC':					tuple_msg[10],
		'LightIR_ADC':					tuple_msg[11],
		'GEIGER_OUTPUT':				tuple_msg[12],
		'RUNTIME_SEC':					tuple_msg[13]			
		
		}	
		result = test_readings.insert_one(test_readings_data)
		#Uncomment to test
		print('One post: {0}'.format(result.inserted_id))
		data_row = []	

if __name__ =='__main__':
	mqttc = mqtt.Client("Weather_Station_Calit2_Server")
	mqttc.on_message = on_message
	mqttc.on_connect = on_connect

	user = 'xxxxxxxxxxxxxx'
	password = 'xxxxxxxxxxxxxx'
	port = 14668
	broker_address = 'm12.cloudmqtt.com'

	mqttc.username_pw_set(user, password=password)
	mqttc.connect(broker_address, port=port)

	mqttc.subscribe("Calit2_Weather_Station/#")
	mqttc.loop_forever()
