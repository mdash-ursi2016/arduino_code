import serial
import re
from time import sleep

ser = serial.Serial('/dev/ttyACM2', 115200)

infile = open('sample_ecg.txt', 'r') 

ECGs = []
for line in infile:
	ECGs.append(line)

for ecg in ECGs:
	ser.write(ecg)

while 1:
	print(ser.readline())