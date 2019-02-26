###
#   Jacob Waller
#   2/26/19
#   This will read data from the serial port and write to a file
###

import serial
import sys

#Will need to change this COM port depending on which port is being used
ser = serial.Serial('COM24', 9600)

writeFile = open("JacobWHeelWalking.csv", "w+")

# Prints data to the terminal and 
# MM LF MF HEEL ACC

#Do this forever
while True:
    #Do this while there's data waiting in the Serial Buffer
    while ser.in_waiting:
        try:
            t = ser.readline()
            t = t.decode('utf-8')
            print(t)
            t = t.replace("\t",",") #Turn the tsv into csv
            t = t.replace("\n", "") #get rid of newlines
            writeFile.write(t)
        except KeyboardInterrupt: #If ctrl+c is pressed, close the file, then exit
            writeFile.close()
            sys.exit()