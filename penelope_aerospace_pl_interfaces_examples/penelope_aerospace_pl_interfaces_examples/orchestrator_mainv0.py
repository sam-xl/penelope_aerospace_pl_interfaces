# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 15:50:22 2023

@author: schutte

"""


"""



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

