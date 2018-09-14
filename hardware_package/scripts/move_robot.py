#! /usr/bin/env python

import rospy
from hardware_package.communication_mainboard import ComportMainboard

if __name__ == '__main__':
	mainboard_runner = ComportMainboard()
	mainboard_runner.run()
	mainboard_runner.write("r/n")
