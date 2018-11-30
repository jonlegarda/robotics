#!/usr/bin/env python
import rospy
from hardware_package.communication_mainboard import ComportMainboard
from general_package.msg import MoveSpeed

# field_id for referee commands.
FIELD_ID = "A"
# robot id for referee commands.
ROBOT_ID = "A"

THROWER_NOT_RUNNING_SPEED = 1000

class MainboardRunner():

    board = None

    front_left_speed = 0
    front_right_speed = 0
    back_speed = 0
    thrower_speed = THROWER_NOT_RUNNING_SPEED

    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("move_speed", MoveSpeed, self.callback)
        self.board = ComportMainboard()
        self.robot_running = True

    def run(self):
        print("Started")
        self.board.run()
        #rospy.spin()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.send_speeds_to_mainboard()
            self.board.read()

            rate.sleep()
        self.board.close()
        print("board is closing...")

    def send_speeds_to_mainboard(self):
        self.board.write("d:{}\n".format(self.thrower_speed))
        self.board.write("sd:{}:{}:{}\n".format( self.front_right_speed, self.back_speed, self.front_left_speed))

    def callback(self, speeds):
        if self.robot_running:
            print(str(speeds))
            self.front_left_speed = speeds.l
            self.front_right_speed = speeds.r
            self.back_speed = speeds.b
            self.thrower_speed = speeds.t

    # referee commands task.
    def referee_commands(self):
        print("ref_cmd")
        line = self.board.readLine()
        print("Referee Commands - read line: " + line)
        if line and line.startswith("<ref:a") and line[6] == FIELD_ID and (line[7] == ROBOT_ID or line[7] == "X"):
            print("Referee Command --> " + line)
            if line.startswith("START", 8):
                self.robot_running = True
            elif line.startswith("STOP", 8):
                self.robot_running = False
                self.front_left_speed = 0
                self.front_right_speed = 0
                self.back_speed = 0
                self.thrower_speed = THROWER_NOT_RUNNING_SPEED
            elif not line.startswith("PING", 8):
                return
            if line[8] == ROBOT_ID:
                self.board.write("ref:a{}{}ACK------\n".format(FIELD_ID, ROBOT_ID))

if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass