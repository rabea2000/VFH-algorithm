#!/usr/bin/env python3
import rospy 
import numpy as np
import math
from math import pi 
from nav_msgs.msg import Odometry , OccupancyGrid 
import matplotlib.pyplot as plt
import  tf 
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Float64MultiArray

import time

class vfh :
    def __init__(self):



        self.ws = 25 # shape of active region
        self.region = np.zeros((self.ws , self.ws) , dtype=np.int8) 
    
       
        self.dmax = ((self.ws - 1) / 2 ) * math.sqrt(2)#distance between the farthest active cell and the VCP 
        self.a = 2
        self.b = self.a / self.dmax  
        
        self.vcp = 13 
        self.sector = 73
        self.rang_of_sector = 360 / (self.sector -1 )
         
        self.l = 4

        self.threshold = 5000 

        self.goal_x = 5
        self.goal_y = 5
        self.smax = 14
        self.obstacle = True
        self.free = False
        self.move = True



        self.v_max = 0.25
        self.v_min = 0.01
        self.angular_max = 2 
        self.vel = 0.2
               


        self.width  = 25
        self.height = 25
        self.resolution = 0.2    
        
  

        self.end = False
        rospy.Subscriber("/window_data"  , OccupancyGrid ,callback=self.get_active_region) 
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_his = rospy.Publisher('/histogram', Float64MultiArray, queue_size=10)
        
        

     
    def polar_histogram(self) :
        active_region = self.region
       # calcaulate magnitude of active_region 
        magnitud = self.calcaulate_magnitude(active_region) 

        histogram =np.zeros( self.sector) 
        h , w  = active_region.shape


        for sector in range(73):
            start_sector = sector*self.rang_of_sector
            end_sector = (sector+1 )*self.rang_of_sector 
            

            for i in range(h) :
                for j in range(w) :
                    x , y = self.ij_to_xy(i,j)
                    # print(f'i = {i} j {j} x {x} y {y}')
                

                    angel_of_cell =  math.degrees(self.correct_angle(np.arctan2(y,x)) ) 
                    if (start_sector <= angel_of_cell ) and (angel_of_cell <= end_sector)  : # mean that the cell is with in the sector       
                        histogram[sector] += magnitud[j,i]  
                        # print (f" sector {sector}  angel { int(math.degrees(angel_of_cell))} i  = {i}  j= {j} x= {x} ,y={y} c = {magnitud[j,i]} d ={data_array[j,i]}")
                        
        


        return histogram

        
    def smoothing_histogram(self , histogram):
            
    # smoothing the histogram 
        histogram_smooth = np.zeros( self.sector)
        for i in range(len(histogram)) :
            value = 0 
            constant = 1
            for k in range(-self.l  , self.l) :
                
                value += constant * histogram[(i + k + self.sector ) % self.sector] 
                constant +=1 
                if constant > self.l :
                    constant =1 


            histogram_smooth[i] = value / (2 *self.l + 1 ) 


        return histogram_smooth    
        
       
 



    def algorithm(self ) :
        # if not self.move :
        #     return 


                

        odom_data =  rospy.wait_for_message("/odom", Odometry)         
        distance_to_goal = self.get_distance_to_goal(odom_data, self.goal_x, self.goal_y)   
        print (F"distanc to goal is {distance_to_goal}")
        
        if distance_to_goal < 0.5: 
            print("-----------------reached the goal--------------") 

            self.end = True
            return 



        histogram_smooth = self.smoothing_histogram(self.polar_histogram())
        
        histogram_binary = histogram_smooth  >= self.threshold       


        msg = Float64MultiArray()
        
       
        msg.data = histogram_binary
        self.pub_his.publish(msg)
  
        pos = odom_data.pose.pose.position
        self.robot_x, self.robot_y = pos.x, pos.y
        



        target_sector = int (math.degrees(self.correct_angle(np.arctan2(self.goal_y - self.robot_y , self.goal_x - self.robot_x))) / self.rang_of_sector )
          
        print(f"the k_target is {target_sector} which is {target_sector*self.rang_of_sector}")

        kf , kn , target_sector , goal_sector   = self.select_valleys(target_sector,histogram_binary)


        # here get the angle that robot shoulde follow it 
        goal_sector_angle = (goal_sector * self.rang_of_sector) 




        # control
        self.go_to_angle(goal_sector_angle ) 

   
        
        


        hc_polar = histogram_smooth[goal_sector]
        hm = 15000 
        hc = min (hc_polar , hm)           
        self.vel = self.v_max * (1 - hc / hm)      
        vel_msg = Twist()   
        vel_msg.linear.x  =  self.vel    
        self.pub.publish(vel_msg)    



    def calcaulate_magnitude ( self , window_map) :

        
        M = 1
        
        N, _ = window_map.shape
        vcp = (N - 1) / 2
        distances = np.zeros(window_map.shape)
        d_max = 1.4142135 * vcp
        
        for i in range(N):
            for j in range(N):
                # calculate di,j
                distances[i,j] = ((i - vcp) ** 2 + (j - vcp) ** 2) ** 0.5
        
        #return mi,j
       
        return (window_map ** 2) * (self.a - self.b * distances)

   


    def ij_to_xy (self ,i , j ) :
        i +=1 
        j +=1 
        #1 
        if i >= self.vcp and j <= self.vcp:
            
            return i-self.vcp, self.vcp- j  
            
        #2
        elif i <= self.vcp and j <= self.vcp  :
        
            return i - self.vcp, self.vcp-j 

        # 3 

        elif  i <= self.vcp and j >= self.vcp: 
            
            return i - self.vcp, self.vcp- j 

        # 4    
        elif  i >= self.vcp and j >= self.vcp:
            
            return i- self.vcp , self.vcp- j 


    def correct_angle (self , angle) :
        res = angle
        if angle < 0:
            return angle + (2 * math.pi)
        return angle


    def get_active_region (self , msg:OccupancyGrid) :
        self.region = np.array(list(msg.data)).reshape(self.ws , self.ws)  
        
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
    
    def get_distance_to_goal(self, odom_data: Odometry, goal_x, goal_y):
        pos = odom_data.pose.pose.position
        x, y = pos.x, pos.y
        return math.sqrt((x - goal_x)**2 + (y - goal_y)**2)



    def go_to_angle(self , target ) :
        
        command = Twist() 
        r = rospy.Rate(30)  
        kp = 1.2
        while not rospy.is_shutdown()   :
        
              
            odom_data =  rospy.wait_for_message("/odom", Odometry)   
            distance_to_goal = self.get_distance_to_goal(odom_data , self.goal_x, self.goal_y)
            angle = self.get_current_heading(odom_data)                     

        
            if distance_to_goal < 0.2:  
                print("reached the goal!")  
                command = Twist ()
                command.angular.z =  0
                command.linear.x  =  0         
            
                self.pub.publish(command) 
                self.end = True  
                break  

            target_rad = self.normalize(math.radians(target)) 

            if abs((target_rad - angle)) <= 0.05 :   

                                        
                print("reach to the angle goal ")
                break 
            else :    
                angular_speed  = kp * (target_rad - angle) 
                angular_speed = self.saturate(angular_speed, - self.angular_max, self.angular_max)  
                command.linear.x = self.vel *(1- angular_speed / self.angular_max ) + self.v_min
                command.angular.z = angular_speed
                print(f"target={math.degrees(target_rad)} current:{math.degrees(angle)}")   

            
            
            
            self.pub.publish(command)                  

            
            r.sleep() 
                        
    def saturate(self,value, min, max):
        if value <= min: return(min)
        elif value >= max: return(max)
        else: return(value)


    def normalize( self , angle):
        res = angle
        while res > pi: 
            res -= 2.0 * pi 
        while res < -pi: 
            res += 2.0 * pi 
        return res     

    def get_current_heading(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return float("{:.2f}".format(yaw))

    def get_target_heading(self, odom_data: Odometry, goal_x, goal_y):
        pos = odom_data.pose.pose.position
        current_x, current_y = pos.x, pos.y
        return self.correct_angle(math.atan2((goal_y - current_y), (goal_x - current_x)) )



    def select_valleys (self , target_sector , histogram_binary):
        valleys = histogram_binary 
        i = target_sector - 1        
        j = target_sector + 1         

        kn,kf = None,None    
        goal_sector = None  
        

        if  valleys[target_sector] == self.obstacle :
            # rospy.loginfo(" obstacle in front of you ")
            

            # here will start scan from left to right and the first condicate velleye will  see it will take it  
            while i >= 0 or j < len(valleys):
                res = []
                #start scanning from left 
                
                if i >=0 and valleys[i] == self.free: 
                    ind = i
                    kn = i
                    valley_size = 1
                    while ind >= 0 and valleys[ind] == self.free :
                        valley_size += 1
                        ind -= 1
                    if valley_size >= self.smax:
                        kf = kn - self.smax
                    else:
                        kf = ind + 1
                        
                    
                    goal_sector = (kf + kn) // 2
                    res.append((kf , kn , target_sector , goal_sector))

                # in case it doesnot find valleye
                i -= 1
                
                if j < len(valleys) and valleys[j]  == self.free :
                    ind = j
                    kn = j
                    valley_size = 1
                    while ind < len(valleys) and valleys[ind] == self.free :
                        valley_size += 1
                        ind += 1
                    if valley_size >= self.smax:
                        kf = kn + self.smax 
                    else:
                        kf = ind - 1 
                    
                    
                    goal_sector = (kf + kn) // 2  
                    res.append((kf , kn , target_sector , goal_sector)) 
                    
                j += 1
                
                if len(res) == 2:
                    (kf , kn , target_sector , goal_sector) = res[0] if abs(res[0][0] - res[0][1]) > abs(res[1][0] - res[1][1]) else res[1]
                    break
                elif len(res) == 1:
                    (kf , kn , target_sector , goal_sector) = res[0]
                    break

        # in case where velley in target            
        else:
            # rospy.loginfo("the target is velley")
            res = []
            ind = i 
            kn = i 
            valley_size = 1
            while ind >= 0 and valleys[ind] == self.free:
                valley_size += 1
                ind -= 1
            if valley_size >= self.smax :
                kf = kn - self.smax
            else:
                kf = ind + 1
                
            
            goal_sector = (kf + kn) // 2
            res.append((kf , kn , target_sector , goal_sector))
            
            ind = j  
            kn  = j 
            valley_size = 1
            while ind < len(valleys) and valleys[ind] == self.free :
                valley_size += 1    
                ind += 1  
            if valley_size >= self.smax :        
                kf = kn + self.smax     
            else:  
                kf = ind - 1         
            
            
            goal_sector = (kf + kn) // 2 
            res.append((kf , kn , target_sector , goal_sector))    

            if len(res) == 2   : 
                    (kf , kn , target_sector , goal_sector) = res[0] if abs(res[0][0] - res[0][1]) > abs(res[1][0] - res[1][1]) else res[1]
                    
            elif len(res) == 1 :  
                (kf , kn , target_sector , goal_sector) = res[0] 

        return  kf , kn , target_sector , goal_sector        


if __name__ == "__main__" :
    rospy.init_node("vfh")
    go_to_goal = vfh()
    rospy.sleep(2)
    r = rospy.Rate(25)
    while not rospy.is_shutdown() and  not go_to_goal.end :
        go_to_goal.algorithm()
        r.sleep()


    command =Twist()
    command.angular.z =  0
    command.linear.x  =  0         
    
    go_to_goal.pub.publish(command) 
 




