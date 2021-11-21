#!/usr/bin/env python

from __future__ import print_function
import threading
#import roslib; roslib.load_manifest('teleop_Wrench_keyboard')
import numpy
import rospy

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Accel
from rospy.numpy_msg import numpy_msg
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        i    
   j    k    l
        ,    

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(225.95502297158288,0.15421947398322708,-259.71864774836126,-1.192296123851948,-7.092364942094861,1.7337462815597349),
        'j':(-63.025456177632435,7.822120055167106,-267.86779506231426,-95.67367050456507,-105.12104693605104,627.9102),
        'k':(0.08393458602311327, 0.061023056144267805, -259.6736057178017, -1.2319847662303416, -1.3447775702096387, -0.000),
        'l':(-63.0013229133316,7.503491428022542,-251.58839859727297,12.828285058134803,58.35202497660129,-627.215439418936),
        ',':(-221.4261672158373,-0.4805009507736606,-259.65521573278374,10.408320478408465,36.05709138167108,1.7260778672525203),
    }


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('thruster_manager/input', Wrench, queue_size = 1)
        self.f_x = 0.08393458602311327
        self.f_y = 0.061023056144267805
        self.f_z = -259.6736057178017
        self.t_x = -1.2319847662303416
        self.t_y = -1.3447775702096387
        self.t_z = -0.0007164890183652664
        
        status = 0
        #self.th = 0.0
        #self.speed = 0.0
        #self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        
        self.ready = False
        self.mass = 1.
        self.inertial_tensor = numpy.identity(3)
        self.mass_inertial_matrix = numpy.zeros((6, 6))
        
        if not rospy.has_param("pid/mass"):
            raise rospy.ROSException("UUV's mass was not provided")

        if not rospy.has_param("pid/inertial"):
            raise rospy.ROSException("UUV's inertial was not provided")

        self.mass = rospy.get_param("pid/mass")
        self.inertial = rospy.get_param("pid/inertial")
        
        # update mass, moments of inertia
        self.inertial_tensor = numpy.array(
          [[self.inertial['ixx'], self.inertial['ixy'], self.inertial['ixz']],
           [self.inertial['ixy'], self.inertial['iyy'], self.inertial['iyz']],
           [self.inertial['ixz'], self.inertial['iyz'], self.inertial['izz']]])
        self.mass_inertial_matrix = numpy.vstack((
          numpy.hstack((self.mass*numpy.identity(3), numpy.zeros((3, 3)))),
          numpy.hstack((numpy.zeros((3, 3)), self.inertial_tensor))))

        print(self.mass_inertial_matrix)
        self.ready = True
        
        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self,f_x,f_y,f_z, t_x, t_y, t_z):
        self.condition.acquire()
        
        # extract 6D accelerations (linear, angular) from message
        #linear = numpy.array((f_x,f_y,f_z))
        #angular = numpy.array((t_x, t_y,t_z))
        #accel = numpy.hstack((linear, angular)).transpose()

        # convert acceleration to force / torque
        #force_torque = self.mass_inertial_matrix.dot(accel)
        #self.f_x = force_torque[0]
        #self.f_y = force_torque[1]
        #self.f_z = force_torque[2]
        #self.t_x = force_torque[3]
        #self.t_y = force_torque[4]
        #self.t_z = force_torque[5]
        
        self.f_x = f_x
        self.f_y = f_y
        self.f_z = f_z
        self.t_x = t_x
        self.t_y = t_y
        self.t_z = t_z
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0.08393458602311327, 0.061023056144267805, -259.6736057178017, -1.2319847662303416, -1.3447775702096387, -0.0007164890183652664)
        self.join()
    def run(self):
        wrench = Wrench()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)
            
            wrench.force.x = self.f_x 
            wrench.force.y = self.f_y 
            wrench.force.z = self.f_z 
            wrench.torque.x = self.t_x
            wrench.torque.y = self.t_y
            wrench.torque.z = self.t_z
          
            self.condition.release()

            print(wrench)
            self.publisher.publish(wrench)

        #wrench.force.x = 0.1 
        #wrench.force.y = 0 
        #wrench.force.z = -260 
        #wrench.torque.x = 2
        #wrench.torque.y = 0
        #wrench.torque.z = 0
        #self.publisher.publish(wrench)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_Wrench_keyboard')
    #speed = rospy.get_param("~speed", 0.5)
    #turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
    pub_thread = PublishThread(repeat)

    f_x = 0.08393458602311327
    f_y = 0.061023056144267805
    f_z = -259.6736057178017
    t_x = -1.2319847662303416
    t_y = -1.3447775702096387
    t_z = -0.0007164890183652664
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(f_x,f_y,f_z,t_x, t_y, t_z)

        print(msg)
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                f_x = moveBindings[key][0]
                f_y = moveBindings[key][1]
                f_z = moveBindings[key][2]
                t_x = moveBindings[key][3]
                t_y = moveBindings[key][4]
                t_z = moveBindings[key][5]
               
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' :
                    continue
                f_x = 0.08393458602311327
                f_y = 0.061023056144267805
                f_z = -259.6736057178017
                t_x = -1.2319847662303416
                t_y = -1.3447775702096387
                t_z = -0.0007164890183652664
                if (key == '\x03'):
                    break
 
            pub_thread.update(f_x,f_y,f_z,t_x,t_y,t_z)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

