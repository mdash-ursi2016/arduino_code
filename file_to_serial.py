# this program sends each line of a text file to the serial monitor
# for an arduino program to read in, and then it prints out the 
# output of that Arduino program to the terminal
# for use with the Arduino sketch bpm_from_file.ino 

import serial

# the name of the arduino serial port, the baud rate
ser = serial.Serial('/dev/ttyACM2', 115200)

# contains a text file with an ECG measurement on each line
infile = open('sample_ecg.txt', 'r') 

# a list of all the ECG measurements in the file
# these all contain \n characters at the end, and
# you should NOT remove the \n characters
ECGs = []
for line in infile:
	ECGs.append(line)

# send all the ECG measurements to the serial monitor
for ecg in ECGs:
	ser.write(ecg)

# After you've sent the data, continuously attempt to read
# data from output from the arduino program. Once all the
# data from this script has been sent to the Arduino sketch, 
# the output of that program can be read here and printed out
# to the terminal 
while 1:
	print(ser.readline())