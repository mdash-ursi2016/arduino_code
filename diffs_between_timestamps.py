import sys
import re

# This program takes a file containing a list of timestamps
# and calculates how far off, on average, the interval between each
# timestamp is from the expected interval, 5000 microseconds

def find_diffs(argv):

	# the file containing all the timestamps
	infile = open(argv[1], 'r')

	# make a list of all the time stamps
	times = []
	for line in infile:
		cleaned = re.sub(r'\n','',line) # get rid of new line characters
		times.append(int(cleaned)) # cast timestamps as ints

	# the changes in time between each time stamp in microseconds
	change = [j-i for i, j in zip(times[:-1], times[1:])]

	# how far off the changes are from what they should be: 5000 microseconds
	nonabsvariation = [5000-x for x in change]

	length = len(nonabsvariation)

	variation = [abs(5000-x) for x in change]

	# best case scenario is 0
	# float() is necessary to force Python to do a floating-point division
	ave_variation = sum(variation) / float(len(variation))

	# printing the results
	print("\non average, the distance between \ntimestamps is off by: ")
	print('\t', ave_variation, "microseconds\n")


find_diffs(sys.argv)

# to run, type the following in the terminal:

# mamcqueen@rsrch3:~/ursi/code$ python3 diffs_between_timestamps.py micros.txt 

# this assumes that micros.txt is the in same location as this program