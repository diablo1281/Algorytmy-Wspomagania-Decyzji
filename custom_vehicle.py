from dronekit import Vehicle


class Wind(object):
	def __init__(self, direction = None, speed = None, speed_z = None):
		self.direction = direction
		self.speed = speed
		self.speed_z = speed_z
	def __str__(self):
		return "WIND: direction = {}, speed = {}, speed_z = {}".format(self.direction, self.speed, self.speed_z)

class CustomVehicle(Vehicle):
	def __init__(self, *args):
		super(CustomVehicle, self).__init__(*args)

		self._wind = Wind()

		@self.on_message('WIND')
		def listener(self, name, message):
			self._wind.direction = message.direction
			self._wind.speed = message.speed
			self._wind.speed_z = message.speed_z

			self.notify_attribute_listeners('wind', self._wind)
		@property
		def wind(self):
			return self._wind