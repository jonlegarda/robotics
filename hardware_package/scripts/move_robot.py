#! /usr/bin/env python
import rospy
from hardware_package.communication_mainboard import ComportMainboard
from general_package.msg import MoveSpeed

class MainboardRunner():
    board = None
    def __init__(self):
	rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("move_speed", MoveSpeed, self.callback)
	self.board = ComportMainboard()

    #def run(self):
        #rospy.init_node("comport_mainboad", anonymous=True)
        # rospy.Subscriber("hardware_module/launcher", Launcher, self.launcher_params_callback)
        #rospy.Subscriber("move_vector", Twist, twist_callback)
        #self.board = ComportMainboard()
        #self.board.run()
        #rate = rospy.Rate(60)
        #while not rospy.is_shutdown():
        #self.sendSpeeds()
	    #self.board.write("r/n")
	    # This line below is supposed to make the robot move by activating 
	    # a couple of wheels and make them move.
	    #self.board.write("sd:-20:20:0:0")
        #rate.sleep()
        #print "closing board"
        #self.board.close()

    #def sendSpeeds(self):
     #   self.board.servo(self.launcherParams.servoPos)
     #   self.board.launch_motor(self.launcherParams.motorSpeed)

    def run(self):
        self.board.run()
	rospy.spin()
        print("closing board")
        self.board.close()

    def callback(self, speeds):
	print(str(speeds))
	self.set_dir(speeds.l, speeds.r, speeds.b, speeds.t)

    def move_forward(self, speed):
        self.set_dir(speed, (-1) * speed, 0)

    def move_backwards(self, speed):
        self.set_dir((-1)*speed, speed, 0)

    def rotate(self, speed):
        self.set_dir(speed, speed, speed)

    def circle(self, speed):
        self.set_dir(0, 0, speed)

    def set_dir(self, front_left, front_right, back, thrower=0):
        self.board.write("sd:{}:{}:{}:{}".format(front_left, front_right, back, thrower))

    def get_dir(self):
        self.board.write('gs')
        return self.board.readline()

if __name__ == '__main__':
    try:
	    mainboard_runner = MainboardRunner()
	    mainboard_runner.run()
	    mainboard_runner.move_forward(10)
    except rospy.ROSInterruptException:
	    pass
