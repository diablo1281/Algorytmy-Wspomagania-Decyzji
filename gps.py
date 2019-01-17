from dronekit import connect, VehicleMode
import os, time, json, sys, mmap, posix_ipc, utils
from console_writer import *

def gps_feeder(vehicle):
	write_header("GPS Feeder starting...")
	memory_gps = posix_ipc.SharedMemory("/GPS_SM", posix_ipc.O_CREAT, size=1024)
	mapfile_gps = mmap.mmap(memory_gps.fd, memory_gps.size)

	semaphore_gps = posix_ipc.Semaphore("/GPS_SEM", posix_ipc.O_CREAT)
	semaphore_gps.release()

	while True:
		try:
			time.sleep(0.2)

			if vehicle.gps_0.fix_type in (0, 1):
				write_warning("No GPS fix!")
				continue

			json_gps = {
				'time' : time.time(),
				'lat' : vehicle.location.global_frame.lat,
				'lon' : vehicle.location.global_frame.lon,
				'alt' : vehicle.location.global_frame.alt,
				'r_alt' : vehicle.location.global_relative_frame.alt,
				'roll' : vehicle.attitude.roll,
				'pitch' : vehicle.attitude.pitch,
				'yaw' : vehicle.attitude.yaw,
			}
			
			semaphore_gps.acquire(0.5)
			utils.write_to_memory(mapfile_gps, json.dumps(json_gps))
			semaphore_gps.release()

		except (KeyboardInterrupt, SystemExit):
			write_info_green("Ending...")
			semaphore_gps.release()
			mapfile_gps.close()
			memory_gps.close_fd()
			semaphore_gps.close()
			exit(0)

		except posix_ipc.BusyError:
			write_error("Busy error")

		except:
			write_error("Error")
			pass
