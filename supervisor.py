import posix_ipc, utils, serial, dronekit, threading, time, mmap, json
from console_writer import *
from library import *
from custom_vehicle import CustomVehicle
from gps import gps_feeder
from mission_manager import mission_manager_main

def create_shm_files():
	memory_gps = posix_ipc.SharedMemory("/GPS_SM", posix_ipc.O_CREAT, size=1024)
	mapfile_gps = mmap.mmap(memory_gps.fd, memory_gps.size)
	memory_cmd = posix_ipc.SharedMemory("/CMD_SM", posix_ipc.O_CREAT, size=1024)
	mapfile_cmd = mmap.mmap(memory_cmd.fd, memory_cmd.size)
	json_gps = {
			'time' : 0,
			'lat' : 0,
			'lon' : 0,
			'alt' : 0,
			'r_alt' : 0,
			'roll' : 0,
			'pitch' : 0,
			'yaw' : 0,
			}

	utils.write_to_memory(mapfile_gps, json.dumps(json_gps))
	mapfile_gps.close()
	memory_gps.close_fd()
	mapfile_cmd.close()
	memory_cmd.close_fd()

	gps_sem = posix_ipc.Semaphore("/GPS_SEM", posix_ipc.O_CREAT)
	cmd_sem = posix_ipc.Semaphore("/CMD_SEM", posix_ipc.O_CREAT)
	gps_sem.release()
	cmd_sem.release()
	gps_sem.close()
	cmd_sem.close()

	camera_queue = posix_ipc.MessageQueue("/CAMERA_Q", posix_ipc.O_CREAT)

	while True:
		try:
			tmp = camera_queue.receive(0.1)
		except:
			break

	camera_queue.close()

def connect_vehicle(connection_string):
	#Connecting
	write_header("Connectiong to the device...")
	vehicle = connect_APM(connection_string, vehicle_class = CustomVehicle)

	return vehicle

####################################################################
#MAIN
write_header("SuperVisor starting...")

create_shm_files()
vehicle = connect_vehicle("127.0.0.1:14550")

#GPS_FEEDER
thread_gps = threading.Thread(target = gps_feeder, args = (vehicle, ))
thread_gps.setDaemon(True)
thread_gps.start()

#MISSION MANAGER
thread_manager = threading.Thread(target = mission_manager_main, args = (vehicle,))
thread_manager.setDaemon(True)
thread_manager.start()

try:
	while True:
		time.sleep(10)
		pass
except (KeyboardInterrupt, SystemExit):
	write_warning("Ending...")
	disconnect_APM(vehicle)
	#led_queue.close()
	exit(0)