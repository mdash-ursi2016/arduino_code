import sys
import re

# takes a text file with comma separated valued and outputs
# a file with a specified column in a specific range

def get_col(argv):

	infile = open(argv[1], 'r') # csv file

	index = int(argv[2]) # which column do you want

	start = int(argv[3]) # what line in the file do you wanna start with

	end = start + 6000 # how many lines of the file to take
	
	# get the specified column
	col = []
	for line in infile:
		cleaned = re.sub(r'\n','',line) 
		split = re.split(',', cleaned)
		col.append(split[index])

	# get the range of the column you want
	sublist = col[start:end]
	
	# write the selection to the output file you specified
	outfile = open(argv[4], 'w')
	for item in sublist:
		outfile.write(item + '\n')

get_col(sys.argv)

# example:

# ~$ python3 get_col.py the_beat_of_my_heart.txt 1 13800 s1.txt

# the 1 means the second column and the 13800 means start at line
# 13800 in that file