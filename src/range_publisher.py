#! /usr/bin/env python

from time import sleep

import busio
import rospy
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from std_msgs.msg import Float32


class RangeReader:
	def __init__(self, spi: busio.SPI, cs: digitalio.DigitalInOut):
		# Define ADC instance which is connected to range sensor
		# This is what we are communicating with over SPI.
		self._mcp = MCP.MCP3008(spi, cs)
		
		self._channel = AnalogIn(self._mcp, MCP.P0)
		
	def read_voltage(self) -> float:
		return self._channel.voltage
		
	def read_range(self) -> float:
		raw_voltage = self.read_voltage()
		
		# TODO: Convert from voltage to distance (calibrate).
		distance = raw_voltage
		return distance

def main():
	rospy.init_node('range_publisher')
	pub = rospy.Publisher('range_state', Float32, queue_size=10)  # Publish on imu_state topic

	# Create SPI bus
	spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

	# Create CS
	cs = digitalio.DigitalInOut(board.D22)
	
	range_reader = RangeReader(spi, cs)

	rate = rospy.Rate(2)  # 2Hz
	range_msg = Float32()

	while not rospy.is_shutdown():
		range_msg = range_reader.read_range()
		pub.publish(range_msg)
		rate.sleep()
		
		
if __name__ == "__main__":
	main()
