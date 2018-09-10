import rospy
import hardware_package.communication_mainboard as cm
from communication_mainboard.py import ComportMainboard

   __name__ == '__main__':
     	 mainboard_runner = ComportMainboard()
         mainboard_runner.run()
	 mainboard_runner.write("r/n")
         mainboard_runner.close()


