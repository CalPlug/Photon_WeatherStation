from pymongo import MongoClient
from collections import defaultdict
import paho.mqtt.client as mqtt
import time
import datetime
import sys
import os

prev_msg = ''
weather_dict = defaultdict(lambda: '')
timer = time.time()
connected_with_result_code_zero = 0
#Improve readability of time

def parse_msg(msg):
	cleanResult = []

	for i in msg:
		if i == '*':
			cleanResult.append('')
		else:
			cleanResult.append(i.decode("ascii"))
	
	#Uncomment to test
	#print(cleanResult)
	return tuple(cleanResult)

def parse_geiger(geiger_str):
	'''Where I stopped'''
	
	if(geiger_str[0] == 'C'):
		geiger_row = geiger_str.split(',')
		return geiger_row
	else:
		return ['']*7

def on_connect(client, userdata, flags, rc):
	global connected_with_result_code_zero
	print("Connected with result code " + str(rc))
	connected_with_result_code_zero += 1
	if connected_with_result_code_zero > 1 :
		print("Disconnected -> restarting server...")
		os.execl(sys.executable, 'python', __file__, *sys.argv[1:])
	
#Add on message callbacks for all topics and construct tuple

def on_message(client, userdata, msg):

	global prev_msg
	global weather_dict
	global timer

	weather_msg = str(msg.payload.decode("utf-8"))
	#Uncomment to test
	#print("Incoming msg payload is : " + demand_msg)
	
	if(msg.topic == 'Weather_Station/GEIGER_OUTPUT'):
		weather_msg = parse_geiger(weather_msg)
		weather_dict['CPS'] = weather_msg[1]
		weather_dict['CPM'] = weather_msg[3]
		weather_dict['uSv/hr'] = weather_msg[5]
		weather_dict['Speed'] = weather_msg[6]
	else:
		 
		weather_dict[msg.topic[16:]] = weather_msg
	
	
	if weather_msg == prev_msg:
		return
	else:
		prev_msg = weather_msg
	
	if(time.time() - timer >= 16.0):

			
		client = MongoClient('localhost', 27017)
		db = client.Weather_Station
		test_readings = db.Weather_Station_Test
		test_readings_data = {
		'utcTime':		  	                datetime.datetime.utcnow(),
		'Temperature_F':				weather_dict['Temperature_F'],
		'Temperature_C':	 			weather_dict['Temperature_C'],
		'Humidity_RH': 	 				weather_dict['Humidity_RH'],
		'Pressure_KPa': 				weather_dict['Pressure_KPa'],
		'DewPoint_C': 					weather_dict['DewPoint_C'],
		'DewPoint_F': 					weather_dict['DewPoint_F'],
		'H2OPartialPressure_KPa':			weather_dict['H2OPartialPressure_KPa'],	 
		'WindSpeed_MPH': 				weather_dict['WindSpeed_MPH'],		
		'WindDirection_DEG':				weather_dict['WindDirection_DEG'],
		'RainFall_INCHES_PerRepPeriod':			weather_dict['RainFall_INCHES_PerRepPeriod'],
		'LightUV_INDEX':				weather_dict['LightUV_INDEX'],
		'LightVIS_ADC':					weather_dict['LightVIS_ADC'],
		'LightIR_ADC':					weather_dict['LightIR_ADC'],
		'CPS':						weather_dict['CPS'],
		'CPM':						weather_dict['CPM'],
		'uSv/hr':					weather_dict['uSv/hr'],
		'Speed':					weather_dict['Speed'].strip('\r\n'),
		'RUNTIME_MIN':					weather_dict['RUNTIME_MIN']			
		
		}	
		result = test_readings.insert_one(test_readings_data)
		print(test_readings_data)
		#Uncomment to test
		print('One post: {0}'.format(result.inserted_id))
		timer = time.time()

if __name__ =='__main__':
	mqttc = mqtt.Client("Weather_Station_Calit2_Server")
	mqttc.on_message = on_message
	mqttc.on_connect = on_connect

	user = 'abtziegr'
	password = 'TLJlAwyK0_NG'
	port = 14668
	broker_address = 'm12.cloudmqtt.com'

	mqttc.username_pw_set(user, password=password)
	mqttc.connect(broker_address, port=port)

	mqttc.subscribe("Weather_Station/#")
	mqttc.loop_forever()
