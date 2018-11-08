#!/usr/bin/env python
import rospy
from hardware_package.communication_mainboard import ComportMainboard
from general_package.msg import MoveSpeed

# field_id for referee commands.
FIELD_ID = "A"
# robot id for referee commands.
ROBOT_ID = "A"

class MainboardRunner():

    #board = None

    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("move_speed", MoveSpeed, self.callback)
        self.board = ComportMainboard()
        self.robot_running = True

    def run(self):
        self.board.run()
        rospy.spin()
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.referee_commands()
            rate.sleep()
        self.board.close()
        print("board is closing...")

    def callback(self, speeds):
        if self.robot_running:
            print(str(speeds))
            self.set_dir(speeds.l, speeds.r, speeds.b, speeds.t)

    # This piece of code is surely not being used in the code. Not sure, so we will leave it this time. Needs to be erase if it is unused...........
    def move_forward(self, speed):
        self.set_dir(speed, (-1) * speed, 0)

    def move_backwards(self, speed):
        self.set_dir((-1)*speed, speed, 0)

    def rotate(self, speed):
        self.set_dir(speed, speed, speed)

    def circle(self, speed):
        self.set_dir(0, 0, speed)
    # Until this part...............

    def set_dir(self, front_left, front_right, back, thrower=0):
        self.board.write("sd:{}:{}:{}:{}\n".format(front_left, front_right, back, 0))
        self.board.read()
        if thrower > 0:
            self.board.write("d:1650\n")
        else:
            self.board.write("d:600\n")
        # Not sure if this will work.
        return self.board.readLine()

    def get_dir(self):
        self.board.write('gs')
        return self.board.readLine()

    # referee commands task.
    def referee_commands(self):
        line = self.board.readLine()
        print("Referee Commands - read line: " + line)
        if line and line.startswith("<rf:a") and line[5] == FIELD_ID and (line[6] == ROBOT_ID or line[6] == "X"):
            print("Referee Command --> " + line)
            if line.startswith("START", 7):
                self.robot_running = True
            elif line.startswith("STOP", 7):
                self.robot_running = False
                self.set_dir(0, 0, 0)
            elif not line.startswith("PING", 7):
                return
            if line[7] == ROBOT_ID:
                self.board.write("rf:a{}{}ACK-----".format(FIELD_ID, ROBOT_ID))

if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass