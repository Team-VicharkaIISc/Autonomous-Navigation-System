#!/usr/bin/env python3

import rospy
#import math
from math import pow, atan2, sqrt
from numpy import array
from geometry_msgs.msg import Twist,Pose2D
from phasespace_msgs.msg import Rigids
from phasespace_msgs.msg import Markers
import tf
    
class PidVelocity():

    def __init__(self):

        rospy.init_node("simple_PID_controller")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
    
        # internal parameters
        self.TargetPositionX = 0
        self.TargetPositionY = 0
        self.Targettheta = 0
        self.Targetang = 0

        self.current_theta = 0
        
        self.integral = 0
        self.error = 0
        self.derivative = 0
        self.then = rospy.Time.now()
    
        # pid parameters
        self.Kp_x = rospy.get_param('~Kp_x',0.6)
        self.Ki_x = rospy.get_param('~Ki_x',0)
        self.Kd_x = rospy.get_param('~Kd_x',0)
        
        self.Kp_y = rospy.get_param('~Kp_y',0.6)
        self.Ki_y = rospy.get_param('~Ki_y',1)
        self.Kd_y = rospy.get_param('~Kd_y',0)

        self.Kp_z = rospy.get_param('~Kp_z',0.4)
        self.Ki_z = rospy.get_param('~Ki_z',0.01)
        self.Kd_z = rospy.get_param('~Kd_z',0)


        self.rate = rospy.get_param('~rate',10)

        self.current_position_x = 0
        self.current_position_y = 0

        self.out_min = rospy.get_param('~vel_threshold',-0.22)
        self.out_max = rospy.get_param('~vel_threshold',0.22)

        self.out_min_z = rospy.get_param('~vel_threshold',-1.0)
        self.out_max_z = rospy.get_param('~vel_threshold',1.0)

        self.position_latest = 0.0
        self.prev_pid_time = rospy.Time.now()
    
        #rospy.logdebug("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f" % (self.nodename, self.Kp, self.Ki, self.Kd))
    
        # publishers and subscriber setup
        
        rospy.Subscriber("/phasespace/markers", Markers, self.positionCallback) 
        rospy.Subscriber("/targetPose", Pose2D, self.targetCallback)
        
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size=10)


    def doPID(self,current,target,a):

        pid_dt_duration = rospy.Time.now() - self.prev_pid_time
        pid_dt = pid_dt_duration.to_sec()

        self.prev_pid_time = rospy.Time.now()

        if a == 1:

            self.error = sqrt(pow((self.TargetPositionX - self.current_position_x), 2) + pow((self.TargetPositionY - self.current_position_y), 2))
            #(target - current) + self.TargetPositionY - self.current_position_y)

            self.integral = self.integral + ( self.error * pid_dt)

            self.derivative = ( self.error - self.previous_error ) / pid_dt
            self.previous_error = self.error

            out = ( self.Kp_x * self.error ) + ( self.Ki_x * self.integral ) + ( self.Kd_x * self.derivative)

            if out > self.out_max:
                out = self.out_max
                self.integral = self.integral - ( self.error * pid_dt)

            elif out < self.out_min:
                out = self.out_min
                self.integral = self.integral - ( self.error * pid_dt)


        elif a == 2:

            self.error = (target - current) 

            self.integral = self.integral + ( self.error * pid_dt)

            self.derivative = ( self.error - self.previous_error ) / pid_dt
            self.previous_error = self.error

            out = ( self.Kp_z * self.error ) + ( self.Ki_z * self.integral ) + ( self.Kd_z * self.derivative)

            if out > self.out_max_z:
                out = self.out_max_z
                self.integral = self.integral - ( self.error * pid_dt)

            elif out < self.out_min_z:
                out = self.out_min_z
                self.integral = self.integral - ( self.error * pid_dt)


        else:
            self.error = (target - current)
    
            self.integral = self.integral + ( self.error * pid_dt)
            
            self.derivative = ( self.error - self.previous_error ) / pid_dt
            self.previous_error = self.error

            out = ( self.Ki_y * self.error ) + ( self.Ki_y * self.integral ) + ( self.Kd_y * self.derivative)

            if out > self.out_max:
                out = self.out_max
                self.integral = self.integral - ( self.error * pid_dt)

            elif out < self.out_min:
                out = self.out_min
                self.integral = self.integral - ( self.error * pid_dt)


        return out,self.error

    def positionCallback(self,msg):
        marker_dyn = msg.markers[33]
        marker_ori = msg.markers[35]

        self.current_position_x = round(marker_dyn.x, 4)
        self.current_position_y = round(marker_dyn.y, 4)

        dy = marker_dyn.y - marker_ori.y
        dx = marker_dyn.x - marker_ori.x

        angle = round(atan2(dy, dx), 3)
        
        self.current_theta = angle
        #_, _, self.current_theta = tf.transformations.euler_from_quaternion([msg.rigids[3].qx, msg.rigids[3].qy, msg.rigids[3].qz, msg.rigids[3].qw])

    def targetCallback(self,msg):
        
        self.TargetPositionX = msg.x
        self.TargetPositionY = msg.y
        self.Targetang = self.computeGoalTheta(self.current_position_x,self.current_position_y,self.TargetPositionX,self.TargetPositionY)


    def spin(self):

            self.r = rospy.Rate(self.rate) 
            self.then = rospy.Time.now()
            
            self.position_x_prev = self.current_position_x
            
            self.then = rospy.Time.now()
            while not rospy.is_shutdown():
                self.spinOnce()
                self.r.sleep()

    def computeGoalTheta(self,current_position_x,current_position_y,TargetPositionX,TargetPositionY):

        angle = atan2(TargetPositionY - current_position_y, TargetPositionX - current_position_x)
        
        return angle

    # function to check if target velocity is reached or not.
    def spinOnce(self):

        self.previous_error = 0.0
        self.prev_vel = 0.0
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0 

        
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.y = 0
        vel_msg.angular.x = 0
        vel_msg.angular.z = 0
        
        flag = 0
        # only do the loop if we've recently recieved a target message
        while not rospy.is_shutdown():
            

            if flag == 1:
                vel_msg.angular.z = 0
            else:   
                vel_msg.angular.z,error_an = self.doPID(self.current_theta,self.Targetang,2)
            
            if abs(error_an) <= 0.003:
                flag = 1
                vel_msg.angular.z = 0
                vel_msg.linear.x,error_x = self.doPID(self.current_position_x,self.TargetPositionX,1)
                
                if abs(error_x) <= 0.005:
                    vel_msg.linear.x = 0
                    break
            '''       


                vel_msg.linear.x,error_x = self.doPID(self.current_position_x,self.TargetPositionX,1) 
                vel_msg.linear.y,error_y = self.doPID(self.current_position_y,self.TargetPositionY,0)
                vel_msg.angular.z = 0
                vel_msg.linear.y = 0

                if (abs(error_x) <= 0.01 and abs(error_y) <= 0.01):
                    vel_msg.linear.y = 0
                    #vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                    break
            '''
            #if error_theta<= 0.01:
            #    vel_msg.angular.z = 0

            '''
            if error_y <= 0.001:
                vel_msg.linear.y = 0
                   
                
                if error_x <= 0.001:
                    vel_msg.linear.x = 0
                    vel_msg.angular.z,error_theta = self.doPID(self.current_theta,self.Targettheta,2)
                    
                    #if error_theta <= 0.001:
                        #vel_msg.linear.x = 0
                        #vel_msg.linear.y = 0
                        #vel_msg.angular.z = 0
                        #break

            
            if (vel_msg.linear.x <= 0.01 and vel_msg.linear.y <= 0.01):
                vel_msg.angular.z = self.doPID(self.current_theta,self.Targettheta,2)
                if vel_msg.angular.z <= 0.01:
                    break
            
            '''
            rospy.loginfo("Velocity Commands (x,y,z) = [{}, {}, {}]".format(vel_msg.linear.x,vel_msg.linear.y,vel_msg.angular.z))
            rospy.loginfo("CurrentPosition (x,y,theta) = [{}, {}, {}]".format(self.current_position_x,self.current_position_y,self.current_theta))
            rospy.loginfo("Target Position (x,y,theta) = [{}, {}, {}]".format(self.TargetPositionX,self.TargetPositionY,self.Targetang))
            rospy.loginfo(" ")

            #rospy.loginfo("Target ang, {}".format(self.Targetang))
            #rospy.loginfo("Curr Theta , {}".format(self.current_theta))

            self.pub_cmd_vel.publish(vel_msg)
            self.r.sleep()


if __name__ == '__main__':
    """ main """
    try:
        pidVelocity = PidVelocity()
        pidVelocity.spin()
    except rospy.ROSInterruptException:
        pass