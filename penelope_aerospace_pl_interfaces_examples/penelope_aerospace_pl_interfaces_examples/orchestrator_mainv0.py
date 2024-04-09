# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 15:50:22 2023

@author: schutte

------------------------------------------------------------------------------------------------------------------------------------------
-------IF DATA OF THE INCOMING ARRAY ON THE PLC IS NOT IN THE CORRECT PLACE, FIRST CHECK IF SOMEWHERE IN THE TO SEND RECIPE --------------
----- Identifier used is: 6969------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------
"""


"""

IP adresses:
Gateway (pc for now?)        192.168.1.1
PLC                          192.168.1.20
Cobot                        192.168.1.21




Flow of program:
0) Establish all connections
    0.1) establish connection between PC-PLC
            Be perfomed by [TCP_connect]             
    0.2) establish connection between PC      --> Done
            Be perfomed by [TCP_connect]      --> Done 
*1) On startup, open recipe selector --> for now fill in base speed & welding current                  This one is not used for Penelope but should be included in normal weld program
    1.1) send recipe to PLC
    1.2) wait for signal from PLC that recipe has been received
    1.3) wait for signal from cobot that recipe has been received
    1.4) close window
2) Open window to fill in standard data about program (user, cell name, datalog file name, temp, hump, etc.)
    2.1) save general data in array for datalog file
            Be performed by [General_info_inputs]
    2.2) close window                                           --> Done
            Be performed by [General_info_inputs]
3) After this window open start selecting screen, with start button, stop button, and other buttons and inputs to use during program
    3.1) Have start button ready during while weld is waiting   --> Done
            Be performed by [weld_control]                      --> Done
    3.2) Have stop button ready      - Fix via DI coming from PLC
4) During process 
    4.1) Save incoming data during welding 
    4.2) Wait for signal from cobot to turn on heat to PLC
    4.3) Wait for conformation from PLC that heat is turned on
    4.4) 
5) After welding has been performed save datalog file
    5.1) Get together all arrays of the datalogging
    5.2) Calculate if target temps were reached and indicate areas where NDI should be performed  --- Not essential for now
    5.3) 



Communications between Cobot and PLC
Heat is on (PLC --> cobot)
Cobot is moving (Cobot --> PLC)

"""



import socket
import tkinter as tk
import time
import struct
import sys
import threading
import tkinter as tk
import time
import General_info_inputs
import socket
import numpy as np
import queue
from General_info_inputs import control_window, recipe_window, processing_recipe
import os
import csv

process_running = False
start = True
stop = False
datalog_ready_for_logging = False



### create datalog file
from csv_creator import create_csv #------------------------- should do something with this------------------------
#from TCP_connect import connect_to_plc, process_PLC_data #------------------------- should do something with this------------------------

#get the correct data from the general info inputs script in this script
Process_start_stop_queue = queue.Queue()
Process_start_stop_queue = General_info_inputs.Process_start_stop_Queue
recipe = General_info_inputs.output_values
recipe = [0]*11
#print(recipe)
recipe [3] = "bla"
currentspeed_integer = 0

# PLC configuration
IP = '192.168.1.20'  # IP address of the PLC
PORT = 2000  # Port configured in the PLC
BUFFER_SIZE = 1024  # Size of the receive buffer
conn = None
#print("Trying to connect to the PLC with IP address: ", IP, " and port: ", PORT)
Length_data_string = 28 + (2 * int(input("What is the amount of TC? (for Penelope give 16)  ")))



#preperation of arrays for outgoing and incoming data for PLC
output_data_plc = [0] * 30 #makes an empty global list for the output data for the plc. Is needed for the "tkinter" GUI screen
data_to_be_sent = [0]*30
for i in range(0, 30):
    data_to_be_sent[i]=i


#------------Standard lines to be put in the  datalog------------------
Data_header = [
    ["SEP=;"],
    ["Weld ID data"],
    ["Welding cell; Operator name; Top Laminate ID; Bottom Laminate ID; Humidity (%); Ambient temperature (°C); Calculation; Comment"],
]
Data_until_recipe =[
    [" "],
    ["Weld settings"],
    ["Mould selected; Configuration selected; Temperature/Time selection"],
    ["Demonstrator mould; Config not filled in yet; Temperature dependent"],
    [" "],
    ["Coupon recipe"],
    ["Start Dwell (s); Stop Dwell (s); Start Offset (mm); Stop Offset (mm); Startzone Length (mm); Startzone Speed (cm/min); Startzone Current (A); Cruise Speed (cm/min); Cruise Current (A); Stopzone Length (mm); Stopzone Speed (cm/min); Stopzone Current (A); Laminate Length (mm); Welding Pressure (bar); Support Pressure (bar); Cooling Program (ul); Pressure Release Time (min); Max Pressure Release Temperature (°C)"],
    ["0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0"],
    [" "],
    ["Demonstrator recipe"],
    ["Weld pressure (bar); Support pressure (bar); Cooling duration (min); Temperature threshold (°C); Start dwell time (s); Stop dwell time (s); Total amount of points; Total amount of zones; First point zone 1; Last point zone 1; Speed zone 1; Weld current zone 1; Dwell time 1; First point zone 2; Last point zone 2; Speed zone 2; Weld current zone 2; Dwell time 2; First point zone 3; Last point zone 3; Speed zone 3; Weld current zone 3; Dwell time 3; First point zone 4; Last point zone 4; Speed zone 4; Weld current zone 4; Dwell time 4; First point zone 5; Last point zone 5; Speed zone 5; Weld current zone 5; Dwell time 5; First point zone 6; Last point zone 6; Speed zone 6; Weld current zone 6; Dwell time 6; First point zone 7; Last point zone 7; Speed zone 7; Weld current zone 7; Dwell time 7; First point zone 8; Last point zone 8; Speed zone 8; Weld current zone 8; Dwell time 8; First point zone 9; Last point zone 9; Speed zone 9; Weld current zone 9; Dwell time 9; First point zone 10; Last point zone 10; Speed zone 10; Weld current zone 10; Dwell time 10; First point zone 11; Last point zone 11; Speed zone 11; Weld current zone 11; Dwell time 11; First point zone 12; Last point zone 12; Speed zone 12; Weld current zone 12; Dwell time 12; First point zone 13; Last point zone 13; Speed zone 13; Weld current zone 13; Dwell time 13; First point zone 14; Last point zone 14; Speed zone 14; Weld current zone 14; Dwell time 14; First point zone 15; Last point zone 15; Speed zone 15; Weld current zone 15; Dwell time 15; First point zone 16; Last point zone 16; Speed zone 16; Weld current zone 16; Dwell time 16; First point zone 17; Last point zone 17; Speed zone 17; Weld current zone 17; Dwell time 17; First point zone 18; Last point zone 18; Speed zone 18; Weld current zone 18; Dwell time 18; First point zone 19; Last point zone 19; Speed zone 19; Weld current zone 19; Dwell time 19; First point zone 20; Last point zone 20; Speed zone 20; Weld current zone 20; Dwell time 20; First point zone 21; Last point zone 21; Speed zone 21; Weld current zone 21; Dwell time 21; First point zone 22; Last point zone 22; Speed zone 22; Weld current zone 22; Dwell time 22; First point zone 23; Last point zone 23; Speed zone 23; Weld current zone 23; Dwell time 23; First point zone 24; Last point zone 24; Speed zone 24; Weld current zone 24; Dwell time 24; First point zone 25; Last point zone 25; Speed zone 25; Weld current zone 25; Dwell time 25; First point zone 26; Last point zone 26; Speed zone 26; Weld current zone 26; Dwell time 26; First point zone 27; Last point zone 27; Speed zone 27; Weld current zone 27; Dwell time 27; First point zone 28; Last point zone 28; Speed zone 28; Weld current zone 28; Dwell time 28; First point zone 29; Last point zone 29; Speed zone 29; Weld current zone 29; Dwell time 29; First point zone 30; Last point zone 30; Speed zone 30; Weld current zone 30; Dwell time 30; First point zone 31; Last point zone 31; Speed zone 31; Weld current zone 31; Dwell time 31; First point zone 32; Last point zone 32; Speed zone 32; Weld current zone 32; Dwell time 32; First point zone 33; Last point zone 33; Speed zone 33; Weld current zone 33; Dwell time 33; First point zone 34; Last point zone 34; Speed zone 34; Weld current zone 34; Dwell time 34; First point zone 35; Last point zone 35; Speed zone 35; Weld current zone 35; Dwell time 35; First point zone 36; Last point zone 36; Speed zone 36; Weld current zone 36; Dwell time 36; First point zone 37; Last point zone 37; Speed zone 37; Weld current zone 37; Dwell time 37; First point zone 38; Last point zone 38; Speed zone 38; Weld current zone 38; Dwell time 38; First point zone 39; Last point zone 39; Speed zone 39; Weld current zone 39; Dwell time 39; First point zone 40; Last point zone 40; Speed zone 40; Weld current zone 40; Dwell time 40; First point zone 41; Last point zone 41; Speed zone 41; Weld current zone 41; Dwell time 41; First point zone 42; Last point zone 42; Speed zone 42; Weld current zone 42; Dwell time 42; First point zone 43; Last point zone 43; Speed zone 43; Weld current zone 43; Dwell time 43; First point zone 44; Last point zone 44; Speed zone 44; Weld current zone 44; Dwell time 44; First point zone 45; Last point zone 45; Speed zone 45; Weld current zone 45; Dwell time 45; First point zone 46; Last point zone 46; Speed zone 46; Weld current zone 46; Dwell time 46; First point zone 47; Last point zone 47; Speed zone 47; Weld current zone 47; Dwell time 47; First point zone 48; Last point zone 48; Speed zone 48; Weld current zone 48; Dwell time 48; First point zone 49; Last point zone 49; Speed zone 49; Weld current zone 49; Dwell time 49; First point zone 50; Last point zone 50; Speed zone 50; Weld current zone 50; Dwell time 50; First point zone 51; Last point zone 51; Speed zone 51; Weld current zone 51; Dwell time 51; First point zone 52; Last point zone 52; Speed zone 52; Weld current zone 52; Dwell time 52; First point zone 53; Last point zone 53; Speed zone 53; Weld current zone 53; Dwell time 53; First point zone 54; Last point zone 54; Speed zone 54; Weld current zone 54; Dwell time 54; First point zone 55; Last point zone 55; Speed zone 55; Weld current zone 55; Dwell time 55; First point zone 56; Last point zone 56; Speed zone 56; Weld current zone 56; Dwell time 56; First point zone 57; Last point zone 57; Speed zone 57; Weld current zone 57; Dwell time 57; First point zone 58; Last point zone 58; Speed zone 58; Weld current zone 58; Dwell time 58; First point zone 59; Last point zone 59; Speed zone 59; Weld current zone 59; Dwell time 59; First point zone 60; Last point zone 60; Speed zone 60; Weld current zone 60; Dwell time 60; First point zone 61; Last point zone 61; Speed zone 61; Weld current zone 61; Dwell time 61; First point zone 62; Last point zone 62; Speed zone 62; Weld current zone 62; Dwell time 62; First point zone 63; Last point zone 63; Speed zone 63; Weld current zone 63; Dwell time 63; First point zone 64; Last point zone 64; Speed zone 64; Weld current zone 64; Dwell time 64; First point zone 65; Last point zone 65; Speed zone 65; Weld current zone 65; Dwell time 65; First point zone 66; Last point zone 66; Speed zone 66; Weld current zone 66; Dwell time 66; First point zone 67; Last point zone 67; Speed zone 67; Weld current zone 67; Dwell time 67; First point zone 68; Last point zone 68; Speed zone 68; Weld current zone 68; Dwell time 68; First point zone 69; Last point zone 69; Speed zone 69; Weld current zone 69; Dwell time 69; First point zone 70; Last point zone 70; Speed zone 70; Weld current zone 70; Dwell time 70; First point zone 71; Last point zone 71; Speed zone 71; Weld current zone 71; Dwell time 71; First point zone 72; Last point zone 72; Speed zone 72; Weld current zone 72; Dwell time 72; First point zone 73; Last point zone 73; Speed zone 73; Weld current zone 73; Dwell time 73; First point zone 74; Last point zone 74; Speed zone 74; Weld current zone 74; Dwell time 74; First point zone 75; Last point zone 75; Speed zone 75; Weld current zone 75; Dwell time 75; First point zone 76; Last point zone 76; Speed zone 76; Weld current zone 76; Dwell time 76; First point zone 77; Last point zone 77; Speed zone 77; Weld current zone 77; Dwell time 77; First point zone 78; Last point zone 78; Speed zone 78; Weld current zone 78; Dwell time 78; First point zone 79; Last point zone 79; Speed zone 79; Weld current zone 79; Dwell time 79; First point zone 80; Last point zone 80; Speed zone 80; Weld current zone 80; Dwell time 80; First point zone 81; Last point zone 81; Speed zone 81; Weld current zone 81; Dwell time 81; First point zone 82; Last point zone 82; Speed zone 82; Weld current zone 82; Dwell time 82; First point zone 83; Last point zone 83; Speed zone 83; Weld current zone 83; Dwell time 83; First point zone 84; Last point zone 84; Speed zone 84; Weld current zone 84; Dwell time 84; First point zone 85; Last point zone 85; Speed zone 85; Weld current zone 85; Dwell time 85; First point zone 86; Last point zone 86; Speed zone 86; Weld current zone 86; Dwell time 86; First point zone 87; Last point zone 87; Speed zone 87; Weld current zone 87; Dwell time 87; First point zone 88; Last point zone 88; Speed zone 88; Weld current zone 88; Dwell time 88; First point zone 89; Last point zone 89; Speed zone 89; Weld current zone 89; Dwell time 89; First point zone 90; Last point zone 90; Speed zone 90; Weld current zone 90; Dwell time 90; First point zone 91; Last point zone 91; Speed zone 91; Weld current zone 91; Dwell time 91; First point zone 92; Last point zone 92; Speed zone 92; Weld current zone 92; Dwell time 92; First point zone 93; Last point zone 93; Speed zone 93; Weld current zone 93; Dwell time 93; First point zone 94; Last point zone 94; Speed zone 94; Weld current zone 94; Dwell time 94; First point zone 95; Last point zone 95; Speed zone 95; Weld current zone 95; Dwell time 95; First point zone 96; Last point zone 96; Speed zone 96; Weld current zone 96; Dwell time 96; First point zone 97; Last point zone 97; Speed zone 97; Weld current zone 97; Dwell time 97; First point zone 98; Last point zone 98; Speed zone 98; Weld current zone 98; Dwell time 98; First point zone 99; Last point zone 99; Speed zone 99; Weld current zone 99; Dwell time 99; First point zone 100; Last point zone 100; Speed zone 100; Weld current zone 100; Dwell time 100"],  
]
To_be_logged_header = [   
    ["Datalog:"],
    ["Weld active (ul); Primary Pressure (bar); Secondary Pressure (bar); Cooling Flow (l/min); Pressure flow 1 (l/min); Pressure Flow 2 (l/min); Generator active (ul); Setpoint Current (A); Output Current (A); Power (W); Frequency (kHz); Timer (s); Coil Speed (cm/min); Current Weld zone; TC 1; TC 2; TC 3; TC 4; TC 5; TC 6; TC 7; TC 8; TC 9; TC 10; TC 11; TC 12; TC 13; TC 14; TC 15; TC 16"],
]



def checkIntPLC(number):
    if (number > 32767):
        number = 32767
    elif (number < -32768):
        number = -32768
    return number


#function that communicates witht eh plc
def connect_and_process():
    global conn, output_data_plc, recipe
    #try connecting to the plc
    try:
        # Create a new socket object
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the PLC
        conn.connect((IP, PORT))
        print("Connected to the PLC")
    except Exception as e:
        print("Error while trying to connect to the PLC. Error message:", str(e))
        sys.exit()

    # Process data
    while True:
        time.sleep(0.1)
        #incoming data
        try:
            # Wait for incoming data
            data = conn.recv(BUFFER_SIZE)
        except Exception as e:
            print("Error while receiving data. Error message:", str(e))
            break

        if not data:
            break

        #check correct data length
        if len(data) == Length_data_string:
            #print("Raw data received:", data)
            print("PLC - correct data length received ")
            # Process and send data as needed
            # Modify as per your requirements
        else:
            print("PLC - Received incorrect number of bytes:", len(data))
            print(data)

# Only process values if the received data is correct data length
        if (len(data) == Length_data_string):
            # Process the received data
            num_variables_received_data = int(Length_data_string/2)
            output_data_plc = [] 
            for i in range(num_variables_received_data):
                unpacked_number = int(struct.unpack(">h", data[i*2:i*2+2])[0])
                output_data_plc.append(unpacked_number)
            #print("output_data_plc = " + str(output_data_plc))
            print("output data plc without actual cobot speed is: " + str(output_data_plc))
            output_data_plc[12] = currentspeed_integer #adds actual cobot speed to the outputdata to be datalogged
            


            #write the current data from the plc to the datalog file
            if datalog_ready_for_logging:
                #filename = "datalogs/filename.csv"  # Specify the path to your CSV file1
                with open(csv_file_path, "a") as file:
                        line = ";".join(map(str, output_data_plc)) + "\n"
                        file.write(line)



            # Convert the numbers to bytes in big-endian style
            recipe = General_info_inputs.output_values
            data_sending_to_plc = data_to_be_sent
            #print("recipe 8 = " + str(recipe[8]))
            if recipe [8] != 0:
                data_sending_to_plc [4] = int(recipe [8])       #welding current
                data_sending_to_plc [2] = 6000                  #welding pressure in mbar
                data_sending_to_plc [3] = 3500                  #tool pressure
                data_sending_to_plc [29] = 6969                  #alignment value at the end of datastring
                #print("recipe values loaded for sending of data")
            else:
                data_sending_to_plc = [0] *11
            
            if process_running:
                data_sending_to_plc [0] = 1
            else:
                data_sending_to_plc [0] = 0
            print("data to be sent to plc" + str(data_sending_to_plc))
            #print("Data sent to plc = "+ str(send_data))
            # Example operation on each element of the list
            #for i in range(len(data_sending_to_plc)):
            #    data_sending_to_plc[i] = checkIntPLC(data_sending_to_plc[i] * 2)

            # Convert the numbers to bytes in big-endian style
            send_data = b''
            #print(data_sending_to_plc)
            #for item in data_sending_to_plc:
            #    print(type(item))

            for number_int in data_sending_to_plc:
                send_data += struct.pack(">h", number_int)
            #print(send_data)
            #print()
            #print("Data send to PLC")
            print("Raw: ", send_data)
            #print("Number 1: ", number1)
            #print("Number 2: ", number2)
            #print("Number 3: ", number3)
            #print()
        # Send the response
            #print(data_sending_to_plc)
            conn.send(send_data)
            print("Data sent to plc = "+ str(send_data))
        else:
# Show error message when the number of bytes is not 6
            print("PLC-Thread: Received too little or too much bytes. 30 needed, received: ",len(data))
                # Close the connection when done
    if conn:
        print("Closing the connection")
        conn.close()


# Start the connection and processing in one thread
        
connect_and_process_thread = threading.Thread(target=connect_and_process)
connect_and_process_thread.daemon = True
#connect_and_process_thread.start()


#function that controls the cobot
def Cobot_communication(): 
    host = "192.168.1.21"
    port = 5000
    connection_cobot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection_cobot.connect((host, port))
    while True:
        global startspeed, speedperc, startcommand, stopcommand, currentspeed_integer, currentstatus_integer
        time.sleep(1/10)
        #print("Connection opened on {}".format(port))

        
        ####outgiong data stuff
        #startspeed
        startspeed = 50               #for now does nothing penelope
        xs =startspeed+1000
        str_x = str(xs)
        
        #speed regulation (welding speed=100% of start speed --> y=50)
        speedperc = 50
        ys=speedperc+1000
        str_y = str(ys)
        
        
        #start command
        #value = process_running
        #value = Process_start_stop_queue.get()
        if process_running == True:
            #print("cobot_moving")
            #Process_start_stop_queue.put(True)
            startcommand = 1
            stopcommand = 0
            #print("current queue: ",list(Process_start_stop_queue.queue))
        elif process_running == False:
            #print("ncobot_not_moving")
            #Process_start_stop_queue.queue.clear()
            #Process_start_stop_queue.put(False)
            startcommand = 0
            stopcommand = 1
            print("COBOT - stopcommand given")
            #print("current queue: ",list(Process_start_stop_queue.queue))
        zs=startcommand+1000
        str_z = str(zs)
        #print(str_z)
        
        #stop commmand
        qs=stopcommand+1000
        str_q = str(qs)
    
        x_b = str_x.encode('utf-8')
        y_b = str_y.encode('utf-8')
        z_b = str_z.encode('utf-8')
        q_b = str_q.encode('utf-8')
    
        #data_to_send = b'\x_b\y_b\z_b\q_b'
        # Combine the 4-byte values into a single byte sequence
        combined_bytes = x_b + y_b + z_b + q_b
    
        # Decode the combined bytes into a string
        message_sending = combined_bytes.decode('utf-8')
        encoded_message_sending = message_sending.encode()
        connection_cobot.send(encoded_message_sending)
        #print("COBOT - current message send to cobot: " + str(combined_bytes))
        #print("message sent is: ", encoded_message_sending)
        #connection_cobot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #connection_cobot.connect((host, port))
               
        
        
        ###incoming data stuff
        try:
            # Wait for incoming data
            data_received = connection_cobot.recv(1024)
            data_received_string = data_received.decode('utf-8')
            #print("data received from cobot = " + data_received_string)
        
        except Exception as e:
            # Show an error message if the script can't receive data or the connection is lost
            
            print("Error while receiving data. Error message: ", str(e))
            #break
        except KeyboardInterrupt: 
            # Catch CTRL+C to stop the script (looks like this doesn't work in Windows)
            print("keyboartIntrupt")
            break
        #print("COBOT- data received is: " + str(data_received)) 
        #-----fix why this part is being a pain in the AAA
        #data_received = connection_cobot.recv(1024)  
        #print(data_received)
        #current welding speed mm/s 
        currentspeed_byte = data_received[0:4]
        currentspeed_string = currentspeed_byte.decode('utf-8')
        currentspeed_integer = int(currentspeed_string)-1000
        print("The current speed is", currentspeed_integer)
        
        #status
        currentstatus_byte = data_received[4:8]
        currentstatus_string = currentstatus_byte.decode('utf-8')
        currentstatus_integer = int(currentstatus_string)-1000
        #print("The status integer is", currentstatus_integer)    #start speed cm/min


#function that should update the data on the output screen
def update_data():
    global integer_vars, output_data_plc, process_running
    while process_running:
        time.sleep(0.1)
        print("output data plc as seen by hmi screen", str(output_data_plc))
        for i in range(14, 30):
            integer_vars[i-14].set(output_data_plc[i])
        root.update()  # Update the Tkinter window


#function to make the actual screen16
def Output_screen_data():
    global root, integer_vars
    # Create the main application window
    root = tk.Tk()
    root.title("Output values thermocouple data")

    # Create an IntVar to hold the integer value
    integer_vars = [tk.IntVar() for _ in range(14, 30)]

    # Create a Label widget to display the integer value
    for i in range(14, 30):
        label_text = f"TC{i-13}: "
        label = tk.Label(root, text=label_text)
        label.grid(row=i-14, column=0, sticky="w", pady=5)  # Left column
        value_label = tk.Label(root, textvariable=integer_vars[i-14])
        value_label.grid(row=i-14, column=1, sticky="e", pady=5)  # Right column



#Write the recipe to the datalog file
def Background_decisions_and_data_handling():
    while True:

        #writing start/stop values
        global process_running, data_to_plc, datalog_ready_for_logging
        #startstop
        #time.sleep(0.5)
        process_running = Process_start_stop_queue.get()
        if process_running:
            print("process starting")
            write_recipe_to_datalog()
            datalog_ready_for_logging = True   
        else:
            print("process stopping")

        

def print_run_value():
    while True:
        time.sleep(1)
        if process_running:
            print("process started and on")
        else:
            print("process stopped")

#-------------- still needs to be called in the correct place----------------------!!!!!!!!!!!!!!!!!!!!
def datalog_data():
    global datalog_ready_for_logging
    #datalog stuff
    fffff = True
    #write last line of incoming data to datalog



Output_screen_data()


#function to write datalog in the correct way to be analyzed by our own software
def write_recipe_to_datalog():
    global recipe_datalog, folder_path, file_name, Cell_operator_etc
    print("outputvalue = " + str(General_info_inputs.output_values))
    recipe = General_info_inputs.output_values
    print("recipe = " + str(recipe))
    # Specify the folder path
    folder_path = 'datalogs'

    # Specify the file name
    file_name = recipe[3] + ".csv"

    
    Current_generator = recipe[8] #link to current in recipe sent to PLC
    Weld_speed_recipe = recipe[9]

    Weld_cell = recipe[4]
    operator = recipe[5]
    stringer_number = recipe[10]
    skin_number = recipe[11]
    humidity = recipe[2]
    ambienttemp = recipe[1]
    calc_number = recipe[6]
    comment = recipe[7]
    recipe_datalog = [
        ["6; 6; 1; 90; 0; 0; 900; 1; 1; 900; {}; {}; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0".format(Weld_speed_recipe, Current_generator)],
    ]
    Cell_operator_etc = [
        ["{};{};{};{};{};{};{};{}".format(Weld_cell, operator, stringer_number, skin_number, humidity, ambienttemp, calc_number, comment)],
    ]
    #Cell_operator_etc = [
    #["Penelope; RS; stringerxxx; skinxxx; humidity; ambienttemp; K1149; comment"],
    #]
    # 0   1    2     3        4     5      6        7
    #[16, 14, 30, 'trial', 'PEN', 'RST', 'K12000', 'Nope, not now', '350', '20']
    make_new_datalog_file()
    print("datalog created on" + csv_file_path)


#defines the function that can be used to create datalog files
def create_csv(folder_path, file_name, data):
    global csv_file_path
    #Create a new CSV file in the specified folder with the given data.

    # Combine the folder path and file name
    csv_file_path = os.path.join(folder_path, file_name)



    # Write data to the CSV file
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)


#make the actual datalogs
def make_new_datalog_file():
    global data
    #               Adds both standard data+datalogging data together
    data = [0]*1
    data = np.concatenate((Data_header, Cell_operator_etc, Data_until_recipe, recipe_datalog, To_be_logged_header), axis=0)
    #print(data)

    #               Use the create_csv function from csv_creator.py
    create_csv(folder_path, file_name, data)   # printf("CSV file created.")
    print("CSV file created")


background_thread = threading.Thread(target=Background_decisions_and_data_handling)
update_thread = threading.Thread(target=update_data)
status_program_thread = threading.Thread(target=print_run_value)
thread_cobot = threading.Thread(target=Cobot_communication)



update_thread.daemon = True  # Daemonize thread so it automatically closes when the main program exits


update_thread.start()
connect_and_process_thread.start()
thread_cobot.start()
background_thread.start()
status_program_thread.start()


control_window()

root.mainloop()

