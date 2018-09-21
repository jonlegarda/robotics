#! /usr/bin/env python

import rospy
from hardware_package.communication_mainboard import ComportMainboard
from game_logic_package.msg import Twist

class MainboardRunner():

    board = None
    #launcherParams = Launcher()

    #def launcher_params_callback(self, launcherMsg):
    #  self.launcherParams = launcherMsg

    def run(self):
        rospy.init_node("comport_mainboad", anonymous=True)
        # rospy.Subscriber("hardware_module/launcher", Launcher, self.launcher_params_callback)
        rospy.Subscriber("move_vector", Twist, twist_callback)
        self.board = ComportMainboard()
        self.board.run()

        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
        #self.sendSpeeds()
	    #self.board.write("r/n")
	    # This line below is supposed to make the robot move by activating 
	    # a couple of wheels and make them move.
	    self.board.write("sd:-20:20:0:0")
        rate.sleep()
        print "closing board"
        self.board.close()

    #def sendSpeeds(self):
     #   self.board.servo(self.launcherParams.servoPos)
     #   self.board.launch_motor(self.launcherParams.motorSpeed)

if __name__ == '__main__':
    try:
	mainboard_runner = MainboardRunner()
	mainboard_runner.run()
    except rospy.ROSInterruptException:
	pass
