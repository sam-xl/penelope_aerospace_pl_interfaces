# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 15:54:33 2023

@author: schutte

Code gotten from ChatGPT
"""


# csv_creator.py

import csv
import os
# Sample data to write to the CSV file




def create_csv(folder_path, file_name, data):
    """
    Create a new CSV file in the specified folder with the given data.
    """
    # Combine the folder path and file name
    csv_file_path = os.path.join(folder_path, file_name)



 # Write data to the CSV file
    with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)





   # printf("CSV file created.")
    print("CSV file created")

