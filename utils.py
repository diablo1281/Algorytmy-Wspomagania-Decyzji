import time 
import sys

PY_MAJOR_VERSION = sys.version_info[0]

if PY_MAJOR_VERSION > 2:
	NULL_CHAR = 0
else:
	NULL_CHAR = '\0'

def say(s):
	"""Prints a timestamped, self-identified message"""
	who = sys.argv[0]
	if who.endswith(".py"):
		who = who[:-3]
		
	s = "%s@%1.6f: %s" % (who, time.time(), s)
	print (s)


def write_to_memory(mapfile, s):
	"""Writes the string s to the mapfile"""
	#say("writing %s " % s)
	mapfile.seek(0)
	# I append a trailing NULL in case I'm communicating with a C program.
	s += '\0'
	if PY_MAJOR_VERSION > 2:
		s = s.encode()
	mapfile.write(s)


def read_from_memory(mapfile):
	"""Reads a string from the mapfile and returns that string"""
	mapfile.seek(0)
	s = [ ]
	c = mapfile.read_byte()
	while c != NULL_CHAR:
		s.append(c)
		c = mapfile.read_byte()
			
	if PY_MAJOR_VERSION > 2:
		s = [chr(c) for c in s]
	s = ''.join(s)
	
   # say("read %s" % s)
	
	return s
	
