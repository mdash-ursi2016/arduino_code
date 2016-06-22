import sys
import re

# takes a text file containing a list of time stamps (in microseconds)
# that correspond to the peaks in a heart beat and prints the beats 
# per minute (bpm)

def bpm(argv):

	# open the file containing the time stamps, as specified by the command
	# line argument
	infile = open(argv[1], 'r')

	# the list of time stamps representing each peak
	times = []
	for line in infile:
		cleaned = re.sub(r'\n','',line) # get rid of new line characters
		times.append(int(cleaned)) # cast time stamps as ints

	# make a list of the change in time between each peak
	change = [j-i for i, j in zip(times[:-1], times[1:])]

	# fine average time between peaks
	ave_change = sum(change) / float(len(change))

	# convert microseconds per beat to beats per minute
	bpm = int(60/(ave_change/1000000)) # this is the BPM
	print(bpm, "bpm")

bpm(sys.argv)

# to run: 

# peaks_to_bpm.py some_peaks.txt