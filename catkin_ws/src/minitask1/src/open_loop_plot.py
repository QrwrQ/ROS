#!/usr/bin/env python

import rospy
import math 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pandas as pd
import csv

class open_track():
    plt_x=0    #refresh the  current position
    plt_y=0
    plt_z=0
    li_x=[0]    #use for saving the trajectory
    li_y=[0]
    fig = plt.figure(tight_layout=True)

    plt.xlim((-1,2))
    plt.ylim((-1,2))

    point_ani,=plt.plot(li_x[0],li_y[0],"r-")
    text_pt = plt.text(3.5,0.8,'',fontsize=16)

#refesh the current position
    def callback(self,msg):
        #rospy.loginfo("record")
        self.plt_x=msg.pose.pose.position.x
        self.plt_y=msg.pose.pose.position.y
        self.plt_z=msg.pose.pose.position.z
        #rate=rospy.Rate(10)
        #rate.sleep()


    def test(self): #drive the car to run a circle

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('testcom', anonymous=True)
        #rospy.Subscriber("odom", Odometory, callback)
        rate= rospy.Rate(2)
        com1=Twist()
        #rospy.loginfo("keeping")
        #pub.publish(com1)
        #rate.sleep()
        rospy.Subscriber("odom", Odometry, self.callback)
        k=0
        while not rospy.is_shutdown()|k>=4: 
            k=k+1
            time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - time < rospy.Duration(10).to_sec():   #goforward for 1 meter
                #rospy.loginfo("Moving Forward")
                print("x:",self.plt_x,"y:",self.plt_y,"z:",self.plt_z)
                self.li_x.append(self.plt_x)
                self.li_y.append(self.plt_y)
                com1.linear.x=0.1
                com1.angular.z=0.0
                pub.publish(com1)
                rate.sleep()
            time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - time < rospy.Duration(10).to_sec():   #turn 90 degrees
                #rospy.loginfo("turn")
                print("x:",self.plt_x,"y:",self.plt_y,"z:",self.plt_z)
                com1.linear.x=0.0
                com1.angular.z=math.pi/20
                pub.publish(com1)
                rate.sleep()
    def plot_update(self):
        xx=list(self.li_x)
        yy=list(self.li_y)
        print(xx)
        self.point_ani.set_data(xx,yy)
        #text_pt.set_text("x=%.3f,y=%.3f"%(x[num],y[num]))
        return self.point_ani,self.text_pt,
    
    def plot_trajectory(self):
        #ani=animation.FuncAnimation(fig=self.fig,func=self.plot_update,frames=np.arange(0,200),
                 #           interval=800, blit=True)
        plt.plot(self.li_x,self.li_y)   #draw the trajectory
        fil=open("trace_data.txt","w+")
        print("Im writing")
        for i in range(0,len(self.li_x)):
            print(str(self.li_x[i])+"  "+str(self.li_y[i]))
            fil.write(str(self.li_x[i])+"  "+str(self.li_y[i]))
        fil.close()
        #dataframe = pd.DataFrame({'x':self.li_x,'y':self.li_y})
        #dataframe.to_csv("op_loop_pos",sep=',')
        '''
        with open("open_po.csv",'wb') as f:
            csv_write=csv.writer(f)
            csv_head=["x","y"]
            csv_write.writerow(csv_head)
        '''
        with open("final_po.csv",'a+') as f:
            csv_write=csv.writer(f)
            csv_write.writerow([self.li_x[-1],self.li_y[-1]])
        with open("open_po.csv",'a+') as f:
            csv_write=csv.writer(f)
            for i in range(0,len(self.li_x)):
                csv_write.writerow([self.li_x[i],self.li_y[i]])


        plt.show()

def stop():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('TestStop', anonymous=True)
    com_stop=Twist()
    i=0
    while i==0:
        pub.publish(com_stop)


if __name__ == '__main__':
    try:
        OL=open_track()
        #OL.plot_update()
        OL.test()
        OL.plot_trajectory()
    except rospy.ROSInterruptException:
        stop()
        #pass
