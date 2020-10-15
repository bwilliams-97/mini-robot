#! /usr/bin/env python

from enum import Enum
from typing import List

import rospy
from mini_robot.msg import IMU

from smbus import SMBus

class MpuRegisters(Enum):
	PWR_MGMT_1 = 0x6B
	SMPLRT_DIV = 0x19
	CONFIG = 0x1A
	GYRO_CONFIG = 0x1B
	INT_ENABLE = 0x38
	ACCEL_XOUT_H = 0x3B
	ACCEL_YOUT_H = 0x3D
	ACCEL_ZOUT_H = 0x3F
	GYRO_XOUT_H = 0x43
	GYRO_YOUT_H = 0x45
	GYRO_ZOUT_H = 0x47
	

class Mpu6050IO():
	def __init__(self, bus: SMBus):
		self._bus = bus
		self._device_address = 0x68
		
		self._accel_conversion = 16384.0
		self._gyro_conversion = 131.0
		
		self.init_mpu()
	
	def init_mpu(self) -> None:
		"""
		Configure MPU6050 sensor.
		"""
		# Write to sample rate register
		self._bus.write_byte_data(self._device_address, MpuRegisters.SMPLRT_DIV.value, 7)
		
		# Write to power management register
		self._bus.write_byte_data(self._device_address, MpuRegisters.PWR_MGMT_1.value, 1)
		
		# Write to config register
		self._bus.write_byte_data(self._device_address, MpuRegisters.CONFIG.value, 0)
		
		# Write to gyro config register
		self._bus.write_byte_data(self._device_address, MpuRegisters.GYRO_CONFIG.value, 24)
		
		# Write to interrupt enable register
		self._bus.write_byte_data(self._device_address, MpuRegisters.INT_ENABLE.value, 1)
	
	def read_raw_data(self, register_address: int) -> int:
		"""
		Read raw data from specified register.
		@param register_address: Register to read from.
		@return raw sensor reading from specified register.
		"""
		# Accel and gyro values are 16-bit (so spread over 2 registers)
		high = self._bus.read_byte_data(self._device_address, register_address)
		low = self._bus.read_byte_data(self._device_address, register_address+1)
		
		# Concatenate higher and lower value
		value = ((high << 8) | low)
		
		# Get signed value 
		if value > 32768:
			value -= 65536
			
		return value
		
	def process_raw_data(self, raw_value: int, gyro: bool=False) -> float:
		"""
		Convert from raw sensor output to useful reading.
		@param raw_value: Raw sensor output.
		@param gyro: Set True if converting gyro output. Otherwise assumed accel.
		@return converted sensor reading.
		"""
		if gyro:
			conversion_factor = self._gyro_conversion
		else:
			conversion_factor = self._accel_conversion
			
		return raw_value / conversion_factor
		
	
	def get_processed_sensor_output(self, gyro: bool=False) -> List[float]:
		"""
		Get accelerometer or gyroscope output and store in ImuSensorOutput
		instance.
		@param gyro: Set True if gyro output desired. Otherwise assumed accel.
		@ return processed x,y,z sensor readings.
		"""
		if gyro:
			sensor_registers = [
				MpuRegisters.GYRO_XOUT_H.value, 
				MpuRegisters.GYRO_YOUT_H.value, 
				MpuRegisters.GYRO_ZOUT_H.value
			]
		else:
			sensor_registers = [
				MpuRegisters.ACCEL_XOUT_H.value, 
				MpuRegisters.ACCEL_YOUT_H.value, 
				MpuRegisters.ACCEL_ZOUT_H.value
			]
		
		output = []
		for i, register in enumerate(sensor_registers):
			component = self.read_raw_data(register)
			processed_component = self.process_raw_data(component, gyro)
			output.append(processed_component)
			
		return output


def main():
	rospy.init_node('imu_publisher')
	pub = rospy.Publisher('imu_state', IMU, queue_size=10)  # Publish on imu_state topic

	rate = rospy.Rate(2)  # 2Hz
	imu_msg = IMU()

	# I2C serial bus
	bus = SMBus(1)
	mpu_comms = Mpu6050IO(bus)

	while not rospy.is_shutdown():
		accel_output = mpu_comms.get_processed_sensor_output()
		gyro_output = mpu_comms.get_processed_sensor_output(gyro=True)
		imu_msg.accel_data = accel_output
		imu_msg.gyro_data = gyro_output
		pub.publish(imu_msg)
		rate.sleep()
		

if __name__ == "__main__":
	main()
