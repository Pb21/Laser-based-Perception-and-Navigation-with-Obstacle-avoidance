#!/usr/bin/env python
import numpy
import rospy
import math
import random
import roslib;
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from array import array

def listener():
    rospy.Subscriber("/base_scan", LaserScan, talker)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker(data):
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    rate = rospy.Rate(10) # 10hz
#    print data
    angle_incr = data.angle_increment
    r = data.ranges
    msg = Twist()
    flag=1
    #Detecting crash point
    for i in range(361):
    	if r[i] < 1.0:
	   print "stop"
	   msg.angular.z=0.0
           msg.linear.x=0.0	
	   flag = 0
	   break 
   
    if flag!=0:
    	msg.linear.x = 2.0
    	msg.angular.z = 0.0

    pub.publish(msg)	
    
    points =[]


    #Detecting obstacles    
    k=0
    if flag==0:
	    for j in range(361):
    		if r[j] < 3.0:
		   #print "points detected"

	  	  #keeping track of the indices in the ranges array which are points
		   points.append(j)
		   k=k+1
	
	    max_iterations=20
	    threshold_inliers=0.03
	    result = computeRansac(points,r,k,max_iterations,threshold_inliers,angle_incr,0) 	
	    
#    result=[best_point_1_x,best_point_1_y,best_point_2_x,best_point_2_y]
    print "before RViz"
    print len(result)
    publishToRviz(result)
    rate.sleep()


def computeRansac(points,r,k,max_iterations,threshold_inliers,angle_incr,flag):
            angle_incr = angle_incr
	    max_iterations = max_iterations
	    threshold = threshold_inliers
	    k=k
	    z=2/3*k
	    max_allowed_outliers = 200
#	    print "max_allowed_outliers"
#	    print max_allowed_outliers
#	    print k
	    #for all the points detected, running RANSAC algorithm
	    #max_no_inliers = math.ceil(2/3 *k)
	    max_no_inliers = 0
	    max_no_outliers=0
	    best_point_1 = 0
	    best_point_2 = 0

	    iterations=0
#	    print "STARTS while loop"
	    best_point_1_x = 0
	    best_point_2_x = 0
	    best_point_1_y = 0
	    best_point_2_y = 0
	    while iterations< max_iterations:
	
		index_1, index_2 = random.sample(points,2)
#		print index_1
#		print index_2

	        #two polar points are then r[index_1] and r[index_2]    x=rCostheta y=rSintheta
		r1 = r[index_1]
		r2 = r[index_2]
		angle_1 = index_1*angle_incr
		angle_2 = index_2*angle_incr

#		print angle_1
#		print angle_2
		
		#x1,y1 and x2,y2 are the cartesian co-ordinates of the two randomly selected points
		y1= -r1*numpy.cos(angle_1)
		x1= r1*numpy.sin(angle_1)
	
		y2= -r2*numpy.cos(angle_2)	
		x2= r2*numpy.sin(angle_2)

		
		#Now for the line between these two points, need to find out the distance of the other points from this line
		delta_y = y2-y1
		delta_x = x2- x1
		
		deno = math.sqrt(delta_y**2 +delta_x**2)
		outlier=[]
		inlier= []
	
		for m in range(k):
			if points[m] == index_1 or points[m] == index_1:
				continue
			else:
				current_index = points[m]
				new_polar_point = r[current_index]
				new_theta = current_index*angle_incr
				new_point_x = new_polar_point*numpy.cos(new_theta)
				new_point_y = new_polar_point*numpy.sin(new_theta)		
	
				distance = (math.fabs((delta_y*new_point_x) -(delta_x*new_point_y)-(y2*x1)))/deno
				if distance > threshold:
					#OUTLIER
					outlier.append(points[m])
				else:
					#INLIER
					inlier.append(points[m])	
	
			
		inlierNo =len(inlier)
		outlierNo=len(outlier)	
			
		if inlierNo > max_no_inliers:
			max_no_inliers=inlierNo
			best_point_1 = index_1
			best_point_2 = index_2
			best_point_1_x = x1
			best_point_2_x = x2
			best_point_1_y = y1
			best_point_2_y = y2


		if outlierNo<max_no_outliers:
			max_no_outliers=outlierNo

	
		iterations =iterations+1
		#while loop for inliers end

	    result=[best_point_1_x,best_point_1_y,best_point_2_x,best_point_2_y]
	    print "outside"
	    print len(outlier)	    

	    if flag==0:
		    print "inside"
	            if outlierNo>max_allowed_outliers:
			flag=1
			result_outliers=computeRansac(outlier,r,outlierNo,max_iterations,threshold,angle_incr,flag)
           		print len(result_outliers)
		        result.extend(result_outliers)

	    return result







def publishToRviz(result):
	   topic = 'visualization_marker'
	   publisher = rospy.Publisher(topic, Marker,queue_size=100)
	
	   marker = Marker()
	   marker.header.frame_id = "base_link"
	   marker.header.stamp = rospy.Time.now()
	   marker.type = marker.ARROW
	   marker.scale.x = 0.2
	   marker.color.a = 1.0
	   marker.color.b = 1.0
	   marker.pose.orientation.w = 1.0
	   #marker.pose.position.x = result[0]
	  # marker.pose.position.y = result[1]
	   #marker.pose.position.z = math.cos(count / 30.0) 
	   
           

	   publisher.publish(marker)




if __name__ == '__main__':
    rospy.init_node('evader', anonymous=True)
    while not rospy.is_shutdown():
	listener()
