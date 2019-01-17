
CONST_START = "#> "

#######################################################################

class console_colors:
	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'

#######################################################################

def write_header(text):
	print console_colors.HEADER + CONST_START + console_colors.BOLD + text + console_colors.ENDC

#######################################################################

def write_info_blue(text):
	print console_colors.OKBLUE + CONST_START + console_colors.UNDERLINE + console_colors.BOLD + "INFO:" + console_colors.ENDC + console_colors.OKBLUE + " " + text + console_colors.ENDC

#######################################################################

def write_info_green(text):
	print console_colors.OKGREEN + CONST_START + console_colors.UNDERLINE + console_colors.BOLD + "INFO:" + console_colors.ENDC + console_colors.OKGREEN + " " + text + console_colors.ENDC

#######################################################################

def write_warning(text):
	print console_colors.WARNING + CONST_START + console_colors.UNDERLINE + console_colors.BOLD + "WARNING:" + console_colors.ENDC + console_colors.WARNING + " " + text + console_colors.ENDC

#######################################################################

def write_error(text):
	print console_colors.FAIL +  CONST_START + console_colors.UNDERLINE + console_colors.BOLD + "ERROR:" + console_colors.ENDC + console_colors.FAIL + " " + text + console_colors.ENDC

#######################################################################