import serial
import threading
import time
import subprocess
import rospy

class ComportMainboard(threading.Thread):

    connection = None
    connection_opened = False

    def __init__(self):
        threading.Thread.__init__(self)

    def open(self):
        try:
            ports = subprocess.check_output('ls /dev/ttyACM0', shell=True).split('\n')[:-1]
        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not self.connection_opened and not rospy.is_shutdown():
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                    self.connection_opened = self.connection.isOpen()
                    time.sleep(0.5)
                self.connection.flush()
                print ("mainboard: Port opened successfully")
            except Exception as e:
                print(e)
                continue

        return self.connection_opened

    def write(self, comm):
        if self.connection is not None and self.connection.is_open:
            try:
                self.connection.write(comm)
                # c = ''
                # while c != '\n':
                #     c = self.connection.read()
            except Exception as e:
                print('mainboard: err write ' + comm)
                print(e)

    # def read(self):
    #     c = ""
    #     while not rospy.is_shutdown():
    #         ch = self.connection.read()
    #         if ch == '\n':
    #             break
    #         c = c + ch
    #         print(c)
    #         return c

    def read(self):
        if self.connection is not None and self.connection.is_open:
            while self.connection.in_waiting > 0:
                self.connection.read()

    def close(self):
        if self.connection is not None and self.connection.is_open:  # close coil
            try:
                self.connection.close()
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return

    def readLine(self):
        if self.connection_opened:
            #self.connection.flush()
            response = self.connection.readline()
            print("readLine {}".format(response))

            return response
