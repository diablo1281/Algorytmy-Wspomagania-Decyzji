import dronekit
import math
import time, json
from geopy.distance import distance, lonlat
from pymavlink import mavutil
from console_writer import *


#######################################################################

def connect_APM(connection_string, vehicle_class = None, baud = 115200):
	if vehicle_class is None:
		vehicle = dronekit.connect(connection_string, wait_ready = True, baud = baud)
	else:
		vehicle = dronekit.connect(connection_string, wait_ready = True, vehicle_class = vehicle_class, baud = baud)
	
	return vehicle

#######################################################################

def disconnect_APM(vehicle):
	vehicle.close()
	
	return True
	
#######################################################################

def reconnect_APM(vehicle, connection_string):
	disconnect_APM(vehicle)
	vehicle = connect_APM(connection_string)
	
	return vehicle

#######################################################################

def reboot_APM(vehicle, connection_string, wait_time = 10):
	vehicle.reboot()
	disconnect_APM(vehicle)
	time.sleep(wait_time)
	connect_APM(connection_string)

	return True

#######################################################################

def check_system_status_OK(vehicle):
	if (vehicle.system_status.state == 'STANDBY' or vehicle.system_status.state == 'ACTIVE') and vehicle.is_armable == True and vehicle.ekf_ok is True:
		return True
	else:
		return False

#######################################################################

def check_system_airborne(vehicle):
	if vehicle.system_status.state == 'ACTIVE':
		return True
	else:
		return False

#######################################################################

def do_arm(vehicle):
	if vehicle.is_armable:
		vehicle.armed = True
		while not vehicle.armed:
			time.sleep(0.5)
		return True
	else:
		return False

#######################################################################

def do_disarm(vehicle, force = False):
	#Conditions to be able to disarm
	if vehicle.system_status.state != 'ACTIVE' or force:
		vehicle.armed = False
		return True
	else:
		return False

#######################################################################

def set_mode(vehicle, mode, try_count = 3):
	for x in range(1 , try_count):
		vehicle.mode = dronekit.VehicleMode(mode)
		time.sleep(1)
		if vehicle.mode == dronekit.VehicleMode(mode):
			return True
	return False

#######################################################################

def do_VTOL_takeoff(vehicle, alt):
	if vehicle.mode == dronekit.VehicleMode('GUIDED'):
		msg = vehicle.message_factory.command_long_encode(
			0, 0,											# target_system, target_component
			mavutil.mavlink.MAV_mission_item_NAV_TAKEOFF,		#command
			0,												#confirmation
			0,												# param 1, empty
			0,												# param 2, front transition heading (0 - default, 1 - next waypoint, 2 - takeoff heading, 3 - specified param 4, 4 - any or for the wind)
			0,												# param 3, empty
			0,												# param 4, YAW angle in degrees, NaN for unchanged
			vehicle.location.global_frame.lat,				# param 5, Latitude
			vehicle.location.global_frame.lon,				# param 6, Longitude
			alt)											# param 7, Altitude
		vehicle.send_mavlink(msg)
		return True
	else:
		return False

#######################################################################

def do_VTOL_land(vehicle, land_position):
	if vehicle.mode == dronekit.VehicleMode('GUIDED'):
		
		return True
	else:
		return False

#######################################################################

def do_VTOL_transition(vehicle, fixed_plane = False, VTOL = False):
	if vehicle.mode == dronekit.VehicleMode('GUIDED'):
		if fixed_plane:
			mode = 1
		elif VTOL:
			mode = 2

		msg = vehicle.message_factory.command_long_encode(
			0, 0,										# target_system, target_component
			mavutil.mavlink.MAV_mission_item_DO_VTOL_TRANSITION,		#command
			0,											#confirmation
			mode,										# param 1,
			0,											# param 2
			0,											# param 3
			0,											# param 4
			0,											# param 5,
			0,											# param 6,
			0)											# param 7,
		vehicle.send_mavlink(msg)
		return True
	else:
		return False

#######################################################################

def go_to(vehicle, location):
	if vehicle.mode == dronekit.VehicleMode('GUIDED'):
		vehicle.simple_goto(location)
		return True
	else:
		return False

#######################################################################
#This method is an approximation, and will not be accurate over large distances and close to the earth's poles.
def get_distance_metres_simple(aLocation1, aLocation2):
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	dalt = math.fabs(aLocation2.alt - aLocation1.alt)
	d = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

	return math.sqrt(math.pow(d, 2) + math.pow(dalt, 2))

#######################################################################

def get_distance_meters(aLocation1, aLocation2):
	#Haversine method
	"""R = 6371000 #Radius of "spherical" earth
	lat1_rad = math.radians(aLocation1.lat)
	lat2_rad = math.radians(aLocation2.lat)
	dlat_rad = math.radians(aLocation2.lat - aLocation1.lat)
	dlon_rad = math.radians(aLocation2.lon - aLocation1.lon)
	dalt = math.fabs(aLocation2.alt - aLocation1.alt)

	a = math.sin(dlat_rad / 2) * math.sin(dlat_rad / 2) + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2) * math.sin(dlon_rad / 2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
	d = R * c
	
	d1 = haversine((aLocation1.lat, aLocation1.lon), (aLocation2.lat, aLocation2.lon))

	return (d, d1)"""
	
	#Using Karney method
	d = distance(lonlat(aLocation1.lat, aLocation1.lon, aLocation1.alt), lonlat(aLocation2.lat, aLocation2.lon, aLocation2.alt)).m
	dalt = math.fabs(aLocation2.alt - aLocation1.alt)

	return math.sqrt(math.pow(d, 2) + math.pow(dalt, 2))

#######################################################################

def distance_to_current_waypoint(vehicle):
	next_waypoint = vehicle.commands.next

	if nextwaypoint == 0:
		return None

	mission_item = vehicle.commands[nextwaypoint - 1]

	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z

	targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)

	return distancetopoint

#######################################################################

def download_mission(vehicle, file_name = None):
	write_info_blue("Download mission from vehicle")

	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()

	mission_list = []
	for cmd in cmds:
		mission_list.append(cmd)

	if file_name is not None:
		output = 'QGC WPL 110\n'

		home = vehicle.home_location
		output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)

		for mission_item in mission_list:
			output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (mission_item.seq, mission_item.current,
						mission_item.frame, mission_item.command, mission_item.param1, mission_item.param2, mission_item.param3, mission_item.param4, mission_item.x, mission_item.y, mission_item.z, mission_item.autocontinue)
		with open(file_name, 'w') as file:
			write_info_blue("Saving mission to file: %s" % file_name)
			file.write(output)
			write_info_green("File saved!")
 
	return mission_list

#######################################################################

def upload_mission(vehicle, file_name = None, mission_list = None):
	if file_name is None and mission_list is None:
		write_error("Can't upload mission. Neither file or mission list was given!")
		return False

	if file_name is not None and mission_list is not None:
		write_error("Can't upload mission. Choose only one source!")
		return False

	if file_name is not None:
		mission_list = read_mission(file_name)
		

	if mission_list is not None:
		write_info_blue("Uploading mission to vehicle...")
		cmds = vehicle.commands
		cmds.clear()

		for mission_item in mission_list:
			cmds.add(mission_item)

		cmds.upload()
		write_info_green("Upload done!")
	else:
		write_error("Can't upload mission. List is empty!")
		return False

	return True

#######################################################################

def read_mission(file_name):
	write_info_blue("Reading mission from file: %s" % file_name)

	mission_list = []

	with open(file_name) as file:
		for i, line in enumerate(file):
			if i == 0:
				if not line.startswith('QGC WPL 110'):
					write_error("Given file is not supported version")
					return None
			else:
				line_array = line.split('\t')
				ln_index = int(line_array[0])
				ln_currentwp = int(line_array[1])
				ln_frame = int(line_array[2])
				ln_command = int(line_array[3])
				ln_param1 = float(line_array[4])
				ln_param2 = float(line_array[5])
				ln_param3 = float(line_array[6])
				ln_param4 = float(line_array[7])
				ln_param5 = float(line_array[8])
				ln_param6 = float(line_array[9])
				ln_param7 = float(line_array[10])
				ln_autocontinue = int(line_array[11].strip())

				cmd = dronekit.Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
				mission_list.append(cmd)

	return mission_list

#######################################################################

def add_to_mission(vehicle, new_wp_list, index = None):
	if index is None:
		index = vehicle.commands.next + 1

	mission_list = download_mission(vehicle)

	for new_wp in new_wp_list:
		mission_list.insert(index - 1, new_wp)
		index = index + 1

	upload_mission(vehicle, mission_list = mission_list)
	write_info_green("New WP(s) added!")

#######################################################################

def modify_landing_point(vehicle, lat, lon):
	write_info_blue("Modifing landing point...")
	mission_list = download_mission(vehicle)
	write_info_blue("Old land point: %s, %s" % (str(mission_list[-1].x), str(mission_list[-1].y)))
	mission_list[-1].x = lat
	mission_list[-1].y = lon
	write_info_green("New land point: %s, %s" % (str(mission_list[-1].x), str(mission_list[-1].y)))
	upload_mission(vehicle, mission_list = mission_list)

#######################################################################

def create_waypoint(lat, lon, alt = 0):
	return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt)

#######################################################################

def create_servo_set_command(channel, pwm):
	return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, channel, pwm, 0, 0, 0, 0, 0)

#######################################################################

def reverse_mission(vehicle, file_name = None):
	if file_name is None:
		mission_list = download_mission(vehicle)
	else:
		mission_list = read_mission(file_name)

	write_info_blue("Reversing mission...")
	home = mission_list[0]
	del mission_list[0]
	landing_command = mission_list[-1].command
	takeoff_command = mission_list[0].command

	mission_list[-1].command = takeoff_command
	mission_list[-1].z = mission_list[0].z

	mission_list[0].command = landing_command
	mission_list[0].x = home.x
	mission_list[0].y = home.y
	mission_list[0].z = 0

	mission_list.reverse()

	for i in range(0, len(mission_list) - 1):
		mission_list[i].seq = str(i + 1)

	upload_mission(vehicle, mission_list = mission_list)
	vehicle.commands.next = 1
	write_info_green("Reversed mission uploaded!")
	
	return mission_list

#######################################################################

def offset(vehicle, lat, lon, bearing, distance, height = None):
	if height is not None:
		R = local_earth_radius(lat, height)
	else:
		if vehicle.home_location is None:
			R = local_earth_radius(lat)
			#write_warning("Can't read field elevation!")
		else:
			R = local_earth_radius(lat, height = vehicle.home_location.alt)
			#write_info_blue("Radius with field elevation")
	
	distance = float(distance)
	new_lat = math.degrees(math.asin(math.sin(math.radians(lat)) * math.cos(distance / R) + math.cos(math.radians(lat)) * math.sin(distance / R) * math.cos(math.radians(bearing))))
	new_lon = lon + math.degrees(math.atan2(math.sin(math.radians(bearing)) * math.sin(distance / R) * math.cos(math.radians(lat)), math.cos(distance / R) - math.sin(math.radians(lat)) * math.sin(math.radians(new_lat))))

	return (new_lat, new_lon)

#######################################################################

def clear_mission(vehicle):
	write_info_blue("Clearing mission...")
	cmds = vehicle.commands
	cmds.clear()
	cmds.upload()
	write_info_green("Mission cleared!")

#######################################################################

def create_landing_mission(vehicle, lat, lon, land_now = True):
	approach_distance = 250
	transition_distance = 80
	approach_height = 50
	transition_height = 25

	wind_direction = vehicle._wind.direction
	wind = (wind_direction + 360) % 360
	bearing = (wind + 180) % 360

	if wind_direction >= 0:
		sign = 1
	else:
		sign = -1

	point_count = 3
	point1 = offset(vehicle, lat, lon, bearing - (60 * sign), approach_distance)
	point2 = offset(vehicle, lat, lon, bearing - (30 * sign), approach_distance * 2 / 3)
	point3 = offset(vehicle, lat, lon, bearing, transition_distance)

	mission_list = [create_waypoint(point1[0], point1[1], approach_height), create_waypoint(point2[0], point2[1], approach_height), create_waypoint(point3[0], point3[1], transition_height)]
	add_to_mission(vehicle, mission_list, vehicle.commands.count)
	modify_landing_point(vehicle, lat, lon)
	if land_now is True:
		vehicle.commands.next = vehicle.commands.count - point_count

#######################################################################

def local_earth_radius(lat, height = 0.0):
	R1 = 6378136.6	#equator radius
	R2 = 6356752.0	#pole radius
	lat_rad = math.radians(lat)

	R = math.sqrt((math.pow((math.pow(R1, 2) * math.cos(lat_rad)), 2) + math.pow((math.pow(R2, 2) * math.sin(lat_rad)), 2)) / ((math.pow((R1 * math.cos(lat_rad)), 2) + math.pow((R2 * math.sin(lat_rad)), 2))))
	return R + height

#######################################################################

def angle_of_view(focal, sensor_dimension, degrees = True): #milimeters
	if degrees is True:
		return math.degrees(2 * math.atan(sensor_dimension / (2 * focal)))
	else:
		return 2 * math.atan(sensor_dimension / (2 * focal))

#######################################################################

def meter_per_pixel(height, resolution, focal, sensor_dimension):
	return ((2 * height * math.tan(angle_of_view(focal, sensor_dimension, False) / 2)) / resolution)

#######################################################################

def rotation_correction(vehicle_heading, image_heading):
	return (vehicle_heading + image_heading + 360) % 360

#######################################################################

def vehicle_heading(vehicle):
	return (vehicle.attitude.yaw + 360) % 360

#######################################################################

def calculate_marker_position(vehicle, lat, lon, alt, heading, marker_position = (None, None), image_dimension = (None, None), focal = None, sensor_dimension = (None, None)):
	if None in marker_position:
		write_error("No marker position is given!")
		return False
	if None in image_dimension:
		write_error("No image dimension are given!")
		return False
	if None in sensor_dimension:
		write_error("No sensor dimension are given!")
		return False
	if focal is None:
		write_error("No focal length is given!")
		return False

	x_mpp = meter_per_pixel(alt, image_dimension[0], focal, sensor_dimension[0])
	y_mpp = meter_per_pixel(alt, image_dimension[1], focal, sensor_dimension[1])
	dx = - image_dimension[0] / 2 + marker_position[0]
	dy = image_dimension[1] / 2 - marker_position[1]
	bearing = rotation_correction(heading, (math.degrees(math.atan2(dx, dy)) + 360) % 360)
	distance = math.sqrt(math.pow(dx * x_mpp, 2) + math.pow(dy * y_mpp, 2))

	return offset(vehicle, lat, lon, bearing, distance)

#######################################################################

def create_mapping_mission(vehicle, lat, lon, alt, radius, buffor, margin, overlap, focal, sensor_dimension = (None, None), index = None):
	if None in sensor_dimension:
		write_error("No sensor dimension are given!")
		return False

	width_of_view = field_of_view(alt, focal, sensor_dimension[0])
	count = math.ceil((2 * (radius + margin) / ((width_of_view * (1 - (float(overlap) / 100))))))
	step = (2 * (radius + margin)) / count

	lines = []
	x = -(radius + margin) + (step / 2)
	while x < (radius + margin):
		lines.append(x)
		x += step

	if vehicle.home_location is None:
		height = 0
		write_warning("Can't read field elevation!")
	else:
		height = vehicle.home_location.alt
		write_info_blue("Got field elevation")

	mission_list = []
	heading = vehicle_heading(vehicle)
	for i in range(0, int(count)):
		dx = lines[i]
		dy = radius + buffor
		if (i % 2) is 1:
			dy = -dy
		bearing = rotation_correction(heading, (math.degrees(math.atan2(dx, dy)) + 360) % 360)
		distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
		point = offset(vehicle, lat, lon, bearing, distance, height)
		mission_list.append(create_waypoint(point[0], point[1], alt))

		dy = -dy
		bearing = rotation_correction(heading, (math.degrees(math.atan2(dx, dy)) + 360) % 360)
		distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
		point = offset(vehicle, lat, lon, bearing, distance, height)
		mission_list.append(create_waypoint(point[0], point[1], alt))

	add_to_mission(vehicle, mission_list, index)


#######################################################################

def field_of_view(height, focal, sensor_dimension):
	return 2 * height * math.tan(angle_of_view(focal, sensor_dimension, False) / 2)

#######################################################################

def is_armed(vehicle):
	return vehicle.armed

#######################################################################

def current_mode(vehicle):
	return vehicle.mode

#######################################################################

def is_mode_set(vehicle, mode):
	if vehicle.mode is dronkit.VehicleMode(mode):
		return True
	else:
		return False

#######################################################################

def calculate_marker_matrix(marker_list):
	matrix = [[0.0 for x in range(len(marker_list))] for y in range(len(marker_list))]
	for i in range(len(marker_list)):
		point1 = json.loads(marker_list[i])
		for j in range(i, len(marker_list)):
			if i == j:
				matrix[i][j] = -1
				continue
			point2 = json.loads(marker_list[j])
			matrix[i][j] = get_distance_meters(dronekit.LocationGlobal(float(point1['lat']), float(point1['lon']), 0), dronekit.LocationGlobal(float(point2['lat']), float(point2['lon']), 0))
			

#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################
#######################################################################


