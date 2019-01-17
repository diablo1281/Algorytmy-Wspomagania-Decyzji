from library import *
from console_writer import *
from custom_vehicle import CustomVehicle
import time, dronekit, threading, json, posix_ipc, mmap, utils, os

CONST_MISSION_FILE = "mission.txt"
image_dimension = (1920, 1080)
sensor_dimension = (4.8, 3.6)
focal = 3.67
radius = 100
buffor = 30
margin = 10
overlap = 20	#precentage

def mission_manager_main(vehicle):
	write_header("Mission manager starting...")
	try:
		camera_queue = posix_ipc.MessageQueue('/CAMERA_Q', posix_ipc.O_CREAT)

		memory_cmd = posix_ipc.SharedMemory("/CMD_SM", posix_ipc.O_CREAT, size=1024)
		mapfile_cmd = mmap.mmap(memory_cmd.fd, memory_cmd.size)

		cmd_sem = posix_ipc.Semaphore("/CMD_SEM", posix_ipc.O_CREAT)
		cmd_sem.release()

		markersList = []

		while True:
			received = camera_queue.receive()
			markersList.append(received[0])
			camera_msg = json.loads(received[0])
			marker_point = calculate_marker_position(vehicle, float(camera_msg['lat']), float(camera_msg['lon']), float(camera_msg['r_alt']), float(camera_msg['yaw']), (int(camera_msg['x']), int(camera_msg['y'])), image_dimension, focal, sensor_dimension)
			print "Found new marker at: [", marker_point[0], ", ", marker_point[1], "]"

		write_info_green("Mission done!!!")
		write_warning("Ending...")
		time.sleep(5)
		disconnect_APM(vehicle)
		camera_queue.close()
		mapfile_cmd.close()
		memory_cmd.close_fd()
		cmd_sem.release()
		cmd_sem.close()
		write_warning("BYE")

	except (KeyboardInterrupt, SystemExit):
		write_warning("Ending...")
		disconnect_APM(vehicle)
		camera_queue.close()
		mapfile_cmd.close()
		memory_cmd.close_fd()
		cmd_sem.release()
		cmd_sem.close()
