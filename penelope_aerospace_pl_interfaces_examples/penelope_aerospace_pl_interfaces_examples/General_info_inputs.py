# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 10:34:08 2023

@author: schutte
"""

import tkinter as tk
from tkinter import *
from tkinter import ttk
import threading
import time
import queue



"""

Integrate the following in this script--> originally found in [Get_welding_details_and_recipe]


def cell_operator_etc_data():
    #input data
    Welding_cell = input("What is the name of the welding cell? ")
    Operator = input("Who is the operator? ")
    Top_lam = input("What is the code for the top laminate? ")
    Bottom_lam = input("What is the code for the bottom laminate? ")
    Humidity = input("What is the humidity? ")
    Temp = input("What is the temperature? ")
    Calc_num = input("What is the calculation number? ")
    Comment = input("Fill in any additonal comments ")   
    
    #put all together in single array
    Cell_operator_etc = [
    [Welding_cell+'; '+Operator+'; '+Top_lam+'; '+Bottom_lam+'; '+Humidity+'; '+Temp+'; '+Calc_num+'; '+Comment]
    ]
    
    # Return array back to where function is called
    return Cell_operator_etc


"""
process_running = False
start = False
stop = False
Process_start_stop_Queue = queue.Queue()
output_values = [0]*11


#           ----- main window with buttons to use while process is running -----
def control_window():
    control_window = tk.Tk()
    control_window.title("Controls")
    global entry_int1, entry_int2, entry_int3, entry_str1, entry_str2, entry_str3, entry_str4, entry_str5, output_values, process_running
    
    button_submit = ttk.Button(control_window, text="Fill in recipe", command=recipe_window)
    button_submit.grid(row=11, column=0, columnspan=2, pady=10)    
    
    button_submit = ttk.Button(control_window, text="Start process", command=start_process)
    button_submit.grid(row=12, column=0, columnspan=2, pady=10)
    
    button_submit = ttk.Button(control_window, text="Inactive-stop command", command=stop_process)
    button_submit.grid(row=13, column=0, columnspan=2, pady=10) 
    
    button_exit = ttk.Button(control_window, text="Exit", command=control_window.destroy)
    button_exit.grid(row=14, column=0, columnspan=2, pady=10)
    
    control_window.mainloop()
    #return process_running


#           ----- function to make the recipe window and get the input data -----
def recipe_window(): 
    window = tk.Tk()
    window.title("Input Window")
    global entry_int1, entry_int2, entry_int3, entry_str1, entry_str2, entry_str3, entry_str4, entry_str5, entry_str6, entry_str7, entry_str8, entry_str9, output_values, process_running

    ###              Create main window


    #               Integer inputs
    label_int1 = ttk.Label(window, text="How many TCs(for Penelope use 16):")
    label_int1.grid(row=0, column=0, padx=5, pady=5, sticky=tk.E)
    entry_int1 = ttk.Entry(window)
    entry_int1.grid(row=0, column=1, padx=5, pady=5)
    
    
    label_int2 = ttk.Label(window, text="What is the temperature (C):")
    label_int2.grid(row=1, column=0, padx=5, pady=5, sticky=tk.E)
    entry_int2 = ttk.Entry(window)
    entry_int2.grid(row=1, column=1, padx=5, pady=5)
    
    label_int3 = ttk.Label(window, text="What is the humidity (%):")
    label_int3.grid(row=2, column=0, padx=5, pady=5, sticky=tk.E)
    entry_int3 = ttk.Entry(window)
    entry_int3.grid(row=2, column=1, padx=5, pady=5)
    
    
    #               String inputs
    label_str1 = ttk.Label(window, text="What should the name of the file be (exclude .csv)?:")
    label_str1.grid(row=3, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str1 = ttk.Entry(window)
    entry_str1.grid(row=3, column=1, padx=5, pady=5)
    
    label_str2 = ttk.Label(window, text="Which welding cell is used?:")
    label_str2.grid(row=4, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str2 = ttk.Entry(window)
    entry_str2.grid(row=4, column=1, padx=5, pady=5)
    
    label_str3 = ttk.Label(window, text="What is the name of the operator?:")
    label_str3.grid(row=5, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str3 = ttk.Entry(window)
    entry_str3.grid(row=5, column=1, padx=5, pady=5)
    
    label_str4 = ttk.Label(window, text="What is the calculation code?:")
    label_str4.grid(row=6, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str4 = ttk.Entry(window)
    entry_str4.grid(row=6, column=1, padx=5, pady=5)
    
    label_str5 = ttk.Label(window, text="If you have any additonal comments fill in here:")
    label_str5.grid(row=7, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str5 = ttk.Entry(window)
    entry_str5.grid(row=7, column=1, padx=5, pady=5)

    label_str6 = ttk.Label(window, text="What is the start current?:")
    label_str6.grid(row=8, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str6 = ttk.Entry(window)
    entry_str6.grid(row=8, column=1, padx=5, pady=5) 

    label_str7 = ttk.Label(window, text="What is the speed?:")
    label_str7.grid(row=9, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str7 = ttk.Entry(window)
    entry_str7.grid(row=9, column=1, padx=5, pady=5)  
    
    label_str8 = ttk.Label(window, text="What is the stringer name/number?:")
    label_str8.grid(row=10, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str8 = ttk.Entry(window)
    entry_str8.grid(row=10, column=1, padx=5, pady=5) 

    label_str9 = ttk.Label(window, text="What is the skin name/number?:")
    label_str9.grid(row=11, column=0, padx=5, pady=5, sticky=tk.E)
    entry_str9 = ttk.Entry(window)
    entry_str9.grid(row=11, column=1, padx=5, pady=5)  
    
    
    # Buttons
    button_submit = ttk.Button(window, text="Load_values", command=processing_recipe)
    button_submit.grid(row=11, column=0, columnspan=2, pady=10)
    
    button_exit = ttk.Button(window, text="Exit", command=window.destroy)
    button_exit.grid(row=14, column=0, columnspan=2, pady=10)
    


    # Run the main loop
    window.mainloop()
        

#           ----- translaters the values gotten in the recipe window into array to use elsewhere -----
def processing_recipe():
    global output_values

    int_value1 = int(entry_int1.get())          #amount of TCS
    int_value2 = int(entry_int2.get())          #Temperature
    int_value3 = int(entry_int3.get())          #Humidity
    str_value1 = entry_str1.get()               #File name
    str_value2 = entry_str2.get()               #Name of the welding cell used
    str_value3 = entry_str3.get()               #Operator name
    str_value4 = entry_str4.get()               #Calculation code
    str_value5 = entry_str5.get()               #Additional comments
    str_value6 = entry_str6.get()               #Welding current
    str_value7 = entry_str7.get()               #welding speed
    str_value8 = entry_str8.get()               #stringer name/number
    str_value9 = entry_str9.get()               #skin name/number

    
    output_values = [int_value1, int_value2, int_value3, str_value1, str_value2, str_value3, str_value4, str_value5, str_value6, str_value7, str_value8, str_value9]
    print(output_values)

    # Do something with the values, e.g., print them

#           ----- registers the start process button -----
def start_process():
    start = True
    Process_start_stop_Queue.put(start)
    #print(Process_start_stop_Queue)


    
#           ----- registers the stop process button -----    
def stop_process():
    stop = False
    Process_start_stop_Queue.put(stop)
    #print(Process_start_stop_Queue)




