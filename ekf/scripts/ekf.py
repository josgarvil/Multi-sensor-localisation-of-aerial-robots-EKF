#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose as PoseRobot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray,Pose,PoseStamped
import tf.transformations
import numpy as np
import random
from math import sqrt,exp
import copy
import sympy as sp
from sympy import *

class ekfpose:	
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
	self.z = 0.0
	self.theta = 0.0

class ekfilter:

    def __init__(self):										###
        
        # A subscriber to the topic '/base_pose_ground_truth'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/ground_truth_to_tf/pose',
                                                PoseStamped, self.update_pose)

        # A publisher to the topic 'particles' to publish the cloud of particles
	self.filter_publisher = rospy.Publisher('filter', PoseArray, queue_size = 1)

	self.odom = Odometry()        
	self.odom_prev = Odometry()  
	self.pose = self.odom.pose.pose.position
        self.pose_prev = self.odom.pose.pose.position
	self.vel = self.odom.twist.twist.linear
	self.vel_prev = self.odom.twist.twist.linear
        self.pose_received = False

	self.model_noise = 1.03
        self.range_noise = 0.02
	self.IMU_noise = 0.03
	self.cam_noise = 2.0
	self.d_max = 15.0
	self.T = 0.05 #frecuencia de 2Hz

        self.landmarks  = [[0.2, 1.0, 0.5], [0.0, 0.5, 0.8], [1.5,0.2, 1.0], [1.0, 0.0, 1.8]]

        self.initialize()

       
    def update_pose(self, data):								###
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        
        self.pose.x = round(data.pose.position.x, 4)
        self.pose.y = round(data.pose.position.y, 4)
	self.pose.z = round(data.pose.position.z, 4)
        
	self.vel.x = (self.pose.x - self.pose_prev.x)/self.T
        self.vel.y = (self.pose.y - self.pose_prev.y)/self.T
	self.vel.z = (self.pose.z - self.pose_prev.z)/self.T

        self.odom.pose.pose.orientation.x = data.pose.orientation.x
	self.odom.pose.pose.orientation.y = data.pose.orientation.y
	self.odom.pose.pose.orientation.z = data.pose.orientation.z
	self.odom.pose.pose.orientation.w = data.pose.orientation.w

        if not self.pose_received:
            
            self.pose_prev = copy.deepcopy(self.pose)
	    self.vel_prev = copy.deepcopy(self.vel)
	    self.odom_prev = copy.deepcopy(self.odom)
            self.pose_received = True


    def initialize(self):									###
        """Function to initialize particles in filter."""
	self.ekf = ekfpose()
	self.ekf.x = self.pose.x
	self.ekf.y = self.pose.y
	self.ekf.z = self.pose.z
	self.ekf.theta = 360*random.random()

	"""Sistema de camara"""
	xW = np.array([[1., 0., 0., 1.]])
	yW = np.array([[0., 1., 0., 1.]])
	zW = np.array([[0., 0., 1., 1.]])

	self.psi = 0.90

	wTc = np.array([[np.cos(self.psi+np.pi/4) ,0. ,-np.sin(self.psi+np.pi/4) ,self.pose.x], [np.sin(self.psi+np.pi/4) ,0. ,np.cos(self.psi+np.pi/4) ,self.pose.y], [0. ,-1. ,0. ,self.pose.z], [0. ,0. ,0. ,1.]])
	self.Rx = np.array([[1. ,0. ,0. ,0.], [0. ,np.cos(-np.pi/10) ,-np.sin(-np.pi/10) ,0.], [0. ,np.sin(-np.pi/10) ,np.cos(-np.pi/10) ,0.], [0. ,0. ,0. ,1.]])
	wTc = np.dot(wTc,self.Rx)

	#centroC = np.array([[self.pose.x, self.pose.y, self.pose.z]])
	#xC = np.dot(wTc,xW.T)
	#yC = np.dot(wTc,yW.T)
	#zC = np.dot(wTc,zW.T)

	f = 0.0042
	N = 1500
	M = 1000
	w = 0.00496
	h = 0.00352
	u0 = round(N/2)
	v0 = round(M/2)
	rho_x = w/N
	rho_y = h/M
	fx = f/rho_x
	fy = f/rho_y
	skew = 0.
	self.Kp = np.array([[fx, skew*fx, u0],[0., fy, v0],[0. ,0. ,1.]])

	cTw = np.linalg.inv(wTc)
	#cRw = cTw[0:3, 0:3]
	#ctw = cTw[0:3, 3]

	self.p1 = np.array([[self.landmarks[0][0], self.landmarks[0][1], self.landmarks[0][2] ,1.]])
	self.p2 = np.array([[self.landmarks[1][0], self.landmarks[1][1], self.landmarks[1][2] ,1.]])
	self.p3 = np.array([[self.landmarks[2][0], self.landmarks[2][1], self.landmarks[2][2] ,1.]])
	self.p4 = np.array([[self.landmarks[3][0], self.landmarks[3][1], self.landmarks[3][2] ,1.]])

	p1_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p1.T)
	p2_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p2.T)
	p3_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p3.T)
	p4_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p4.T)
	
	P1 = p1_[0:2]/p1_[2] + random.gauss(0.0, self.cam_noise)
	P2 = p2_[0:2]/p2_[2] + random.gauss(0.0, self.cam_noise)
	P3 = p3_[0:2]/p3_[2] + random.gauss(0.0, self.cam_noise)
	P4 = p4_[0:2]/p4_[2] + random.gauss(0.0, self.cam_noise)

	P1 = np.array([[round(P1[0])],[round(P1[1])]])
	P2 = np.array([[round(P2[0])],[round(P2[1])]])
	P3 = np.array([[round(P3[0])],[round(P3[1])]])
	P4 = np.array([[round(P4[0])],[round(P4[1])]])

	#print('Puntos =',P1,P2,P3,P4)

	self.Q = np.array([[self.range_noise ** 2 ,0. ,0. ,0.], [0. ,self.range_noise ** 2 ,0. ,0.], [0. ,0. ,self.range_noise ** 2 ,0.], [0. ,0. ,0. ,self.range_noise ** 2]])

	self.R = np.array([[self.model_noise ** 2 ,0. ,0. ,0. ,0. ,0.], [0. ,self.model_noise ** 2 ,0. ,0. ,0. ,0.], [0. ,0. ,self.model_noise ** 2 ,0. ,0. ,0.], [0. ,0. ,0. ,self.model_noise ** 2 ,0. ,0.], [0. ,0. ,0. ,0. ,self.model_noise ** 2 ,0.], [0. ,0. ,0. ,0. ,0. ,self.model_noise ** 2]])

	self.estado_est = np.array([[self.pose.x ,self.pose.y ,self.pose.z ,self.vel.x ,self.vel.y, self.vel.z]])

	self.sigma_est = np.array([[self.range_noise ** 2 ,0. ,0. ,0. ,0. ,0.], [0. ,self.range_noise ** 2 ,0. ,0. ,0. ,0.], [0. ,0. ,self.range_noise ** 2 ,0. ,0. ,0.], [0. ,0. ,0. ,self.range_noise ** 2 ,0. ,0.], [0. ,0. ,0. ,0. ,self.range_noise ** 2 ,0.], [0. ,0. ,0. ,0. ,0. ,self.range_noise ** 2]])

	self.estado_est_ant = self.estado_est.copy()
	self.sigma_est_ant = self.sigma_est.copy()
	self.estado_pred = self.estado_est.copy()
	self.sigma_pred = self.sigma_est.copy()

	self.A = np.array([[1. ,0. ,0. ,self.T ,0. ,0.], [0. ,1. ,0. ,0. ,self.T ,0.], [0. ,0. ,1. ,0. ,0. ,self.T], [0. ,0. ,0. ,1. ,0. ,0.], [0. ,0. ,0. ,0. ,1. ,0.], [0. ,0. ,0. ,0. ,0. ,1.]])


	x,y,z,vx,vy,vz = var('x y z vx vy vz', real=True)
	
	fd1 = sqrt((x-self.landmarks[0][0]) ** 2 + (y-self.landmarks[0][1]) ** 2 + (z-self.landmarks[0][2]) ** 2)
	fd2 = sqrt((x-self.landmarks[1][0]) ** 2 + (y-self.landmarks[1][1]) ** 2 + (z-self.landmarks[1][2]) ** 2)
	fd3 = sqrt((x-self.landmarks[2][0]) ** 2 + (y-self.landmarks[2][1]) ** 2 + (z-self.landmarks[2][2]) ** 2)
	fd4 = sqrt((x-self.landmarks[3][0]) ** 2 + (y-self.landmarks[3][1]) ** 2 + (z-self.landmarks[3][2]) ** 2)

	self.h = Matrix([fd1, fd2, fd3, fd4])

	self.H = Matrix([[diff(fd1,x),diff(fd1,y),diff(fd1,z),diff(fd1,vx),diff(fd1,vy),diff(fd1,vz)],[diff(fd2,x),diff(fd2,y),diff(fd2,z),diff(fd2,vx),diff(fd2,vy),diff(fd2,vz)],[diff(fd3,x),diff(fd3,y),diff(fd3,z),diff(fd3,vx),diff(fd3,vy),diff(fd3,vz)],[diff(fd4,x),diff(fd4,y),diff(fd4,z),diff(fd4,vx),diff(fd4,vy),diff(fd4,vz)]])

	print('h =',self.h)
	print('H =',self.H)

	wTck = Matrix([[np.cos(self.psi+np.pi/4) ,0. ,-np.sin(self.psi+np.pi/4) ,x], [np.sin(self.psi+np.pi/4) ,0. ,np.cos(self.psi+np.pi/4) ,y], [0. ,-1. ,0. ,z], [0. ,0. ,0. ,1.]])
	Rxk = Matrix([[1. ,0. ,0. ,0.], [0. ,np.cos(-np.pi/10) ,-np.sin(-np.pi/10) ,0.], [0. ,np.sin(-np.pi/10) ,np.cos(-np.pi/10) ,0.], [0. ,0. ,0. ,1.]])
	wTck = wTck*Rxk
	print('wTck =',wTck)

	cTwk = wTck.inv()
	Kpk = Matrix([[fx, skew*fx, u0],[0., fy, v0],[0. ,0. ,1.]])

	p1_k = Kpk * cTwk[0:3,0:4] * self.p1.T
	p2_k = Kpk * cTwk[0:3,0:4] * self.p2.T
	p3_k = Kpk * cTwk[0:3,0:4] * self.p3.T
	p4_k = Kpk * cTwk[0:3,0:4] * self.p4.T
	
	P1k = p1_k[0:2,:]/p1_k[2,:] #+ random.gauss(0.0, self.cam_noise)
	P2k = p2_k[0:2,:]/p2_k[2,:] #+ random.gauss(0.0, self.cam_noise)
	P3k = p3_k[0:2,:]/p3_k[2,:] #+ random.gauss(0.0, self.cam_noise)
	P4k = p4_k[0:2,:]/p4_k[2,:] #+ random.gauss(0.0, self.cam_noise)

	self.h2 = Matrix([P1k,P2k,P3k,P4k])

	self.H2 = Matrix([[diff(P1k[0,:],x),diff(P1k[0,:],y),diff(P1k[0,:],z),diff(P1k[0,:],vx),diff(P1k[0,:],vy),diff(P1k[0,:],vz)],[diff(P1k[1,:],x),diff(P1k[1,:],y),diff(P1k[1,:],z),diff(P1k[1,:],vx),diff(P1k[1,:],vy),diff(P1k[1,:],vz)],[diff(P2k[0,:],x),diff(P2k[0,:],y),diff(P2k[0,:],z),diff(P2k[0,:],vx),diff(P2k[0,:],vy),diff(P2k[0,:],vz)],[diff(P2k[1,:],x),diff(P2k[1,:],y),diff(P2k[1,:],z),diff(P2k[1,:],vx),diff(P2k[1,:],vy),diff(P2k[1,:],vz)],[diff(P3k[0,:],x),diff(P3k[0,:],y),diff(P3k[0,:],z),diff(P3k[0,:],vx),diff(P3k[0,:],vy),diff(P3k[0,:],vz)],[diff(P3k[1,:],x),diff(P3k[1,:],y),diff(P3k[1,:],z),diff(P3k[1,:],vx),diff(P3k[1,:],vy),diff(P3k[1,:],vz)],[diff(P4k[0,:],x),diff(P4k[0,:],y),diff(P4k[0,:],z),diff(P4k[0,:],vx),diff(P4k[0,:],vy),diff(P4k[0,:],vz)],[diff(P4k[1,:],x),diff(P4k[1,:],y),diff(P4k[1,:],z),diff(P4k[1,:],vx),diff(P4k[1,:],vy),diff(P4k[1,:],vz)]])

	print('h2 =',self.h2)
	print('H2 =',self.H2)

    def check_robot_motion(self):								###
        """Function to determine if robot moved enough to update filter."""

        if not self.pose_received:
            return False

        # Compute movement increment from last time
        self.delta_x = self.pose.x - self.pose_prev.x
        self.delta_y = self.pose.y - self.pose_prev.y
	self.delta_z = self.pose.z - self.pose_prev.z

        # Predict only if the robot moved enough
        if abs(self.delta_x) > 0.2 or abs(self.delta_y) > 0.2 or abs(self.delta_z) > 0.2:
            return True
        else:
            return False


    def sense(self):										###
        """Function to generate sensor observation from measuring distange to landmarks."""
	print('Robot =',self.pose.x,self.pose.y,self.pose.z)
	n1 = -1.0
	n2 = -2.0
        Z = []
        for i in range(len(self.landmarks)):
            dist = sqrt((self.pose.x - self.landmarks[i][0]) ** 2 + (self.pose.y - self.landmarks[i][1]) ** 2 + (self.pose.z - self.landmarks[i][2]) ** 2)
            dist += random.gauss(0.0, self.range_noise)

	    r = random.random()
	    if r <= 0.75:
	    	if dist <= self.d_max:
			Z.append(dist)
		else:
			Z.append(n2)
	    else:
	    	Z.append(n1)

        return Z


    def predict(self):										###
        """Function to predict particles according to robot motion."""

	vxr = self.vel.x + random.gauss(0.0, self.IMU_noise)
	vyr = self.vel.y + random.gauss(0.0, self.IMU_noise)
	vzr = self.vel.z + random.gauss(0.0, self.IMU_noise)

	IncIMU =  np.array([[0.,0.,0.,vxr,vyr,vzr]]) - np.array([[0.,0.,0.,self.estado_est_ant[0][3],self.estado_est_ant[0][4],self.estado_est_ant[0][5]]])
	print('EEA =',self.estado_est_ant)
	print('IncIMU =',IncIMU)

	self.estado_pred = np.dot(self.A,self.estado_est_ant.T) + np.dot(self.A, IncIMU.T)

	self.sigma_pred = np.dot(np.dot(self.A,self.sigma_est_ant),self.A.T) + self.R

	print('Pred =',self.estado_pred)
	print('SPred =',self.sigma_pred)
        #self.pose_prev = copy.deepcopy(self.pose)


    def gaussian(self, mu, sigma, x):								###
            """ calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
            return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * np.pi * (sigma ** 2))


    def update(self, Z):									###	
	
	D1 = self.h.subs([(x,self.estado_pred.conj().T[0][0]),(y,self.estado_pred.conj().T[0][1]),(z,self.estado_pred.conj().T[0][2]),(vx,self.estado_pred.conj().T[0][3]),(vy,self.estado_pred.conj().T[0][4]),(vz,self.estado_pred.conj().T[0][5])])

	D1 = np.array(D1.T)

	C = self.H.subs([(x,self.estado_pred.conj().T[0][0]),(y,self.estado_pred.conj().T[0][1]),(z,self.estado_pred.conj().T[0][2]),(vx,self.estado_pred.conj().T[0][3]),(vy,self.estado_pred.conj().T[0][4]),(vz,self.estado_pred.conj().T[0][5])])

	C = np.array(C)
	#print('C =',C)

	#Act camara
	wTc = np.array([[np.cos(self.psi+np.pi/4) ,0. ,-np.sin(self.psi+np.pi/4) ,self.pose.x], [np.sin(self.psi+np.pi/4) ,0. ,np.cos(self.psi+np.pi/4) ,self.pose.y], [0. ,-1. ,0. ,self.pose.z], [0. ,0. ,0. ,1.]])
	wTc = np.dot(wTc,self.Rx)

	cTw = np.linalg.inv(wTc)

	p1_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p1.T)
	p2_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p2.T)
	p3_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p3.T)
	p4_ = np.dot(np.dot(self.Kp,cTw[0:3,0:4]),self.p4.T)
	
	P1 = p1_[0:2]/p1_[2] + random.gauss(0.0, self.cam_noise)
	P2 = p2_[0:2]/p2_[2] + random.gauss(0.0, self.cam_noise)
	P3 = p3_[0:2]/p3_[2] + random.gauss(0.0, self.cam_noise)
	P4 = p4_[0:2]/p4_[2] + random.gauss(0.0, self.cam_noise)

	P1 = np.array([[round(P1[0])],[round(P1[1])]])
	P2 = np.array([[round(P2[0])],[round(P2[1])]])
	P3 = np.array([[round(P3[0])],[round(P3[1])]])
	P4 = np.array([[round(P4[0])],[round(P4[1])]])

	Z2 =np.array([P1[0],P1[1],P2[0],P2[1],P3[0],P3[1],P4[0],P4[1]]).flatten().tolist()
	print('Puntos =',Z2)


	D2 = self.h2.subs([(x,self.estado_pred.conj().T[0][0]),(y,self.estado_pred.conj().T[0][1]),(z,self.estado_pred.conj().T[0][2]),(vx,self.estado_pred.conj().T[0][3]),(vy,self.estado_pred.conj().T[0][4]),(vz,self.estado_pred.conj().T[0][5])])

	D2 = np.array(D2).flatten().tolist()
	#print('D2 =',D2)

	C2 = self.H2.subs([(x,self.estado_pred.conj().T[0][0]),(y,self.estado_pred.conj().T[0][1]),(z,self.estado_pred.conj().T[0][2]),(vx,self.estado_pred.conj().T[0][3]),(vy,self.estado_pred.conj().T[0][4]),(vz,self.estado_pred.conj().T[0][5])])

	#print('C2 =',C2)
	for i in range(8):
		for j in range(6):
			C2[i,j] = np.array(C2[i,j])
	C2 = np.array(C2)
	#print('C2 =',C2)

	Dlist = []
	Clist = []
	Zlist = []

	#print('Z =',Z)
	#print('D1 =',D1)

	for i in range(len(Z)): 
		if Z[i] >= 0:
			Dlist.append(D1[0][i])
			Clist.append(C[i])
			Zlist.append(Z[i])

	#print('Dlist =',Dlist)
	#print('Zlist =',Zlist)
	Tam1 = len(Zlist)

	for i in range(len(Z2)): 
		if (i>0 and (i % 2)>0 and Z2[i] >= 0 and Z2[i-1] >= 0 and Z2[i] <= 1000 and Z2[i-1] <= 1500):
			Dlist.append(D2[i-1])
			Dlist.append(D2[i])
			Clist.append(C2[i-1])
			Clist.append(C2[i])
			Zlist.append(Z2[i-1])
			Zlist.append(Z2[i])

	#print('Dlist =',Dlist)
	#print('Zlist =',Zlist)

	D = np.array(Dlist)
	C = np.array(Clist)

	TamT = len(Zlist)
	Tam2 = TamT - Tam1
	#print('tamT',len(Zlist))
	#print('tam1',Tam1)
	#print('tam2',Tam2)

	self.Q = np.identity(TamT)*(self.cam_noise ** 2)
	self.Q[0:(Tam1),0:(Tam1)] =np.identity(Tam1)*(self.range_noise ** 2)
	#print('Q =',self.Q)

	Z = np.array(Zlist)
	print('Z =',Z)
	print('D =',D)

	if len(Z) == 0:
		self.estado_est = (self.estado_pred).T
		self.sigma_est = self.sigma_pred
	else:
		k1=np.dot(self.sigma_pred,C.T) #dim 6*6 * 6*x = 6*x
		k2=np.dot(C,self.sigma_pred)	#   x*6 * 6*6 = x*6
		k3=np.dot(k2,C.T)		#   x*6 * 6*x = x*x
		k3= np.float32(k3)
		k4=(k3 + self.Q)		#   x*x + x*_ = x*x
		k4= np.float32(k4)
		K = np.dot(k1,np.linalg.inv(k4))  # 6*x * x*x = 6*x
		#print('K =',K)

		Inn=np.array([Z - D]).T
		self.estado_est = (self.estado_pred + np.dot(K,Inn)).T
		print('Estado =',self.estado_est)

		s1=np.identity(6) - np.dot(K,C)
		self.sigma_est = np.dot(s1,self.sigma_pred)
		print('Sigma =',self.sigma_est)

	self.ekf.x = self.estado_est[0][0]
	self.ekf.y = self.estado_est[0][1]
	self.ekf.z = self.estado_est[0][2]

    	self.estado_est_ant = self.estado_est.copy()
	self.sigma_est_ant = self.sigma_est.copy()


    def publish(self):							###
        """Function to publish the particle cloud for visualization."""
        
        cloud_msg = PoseArray()
        cloud_msg.header.stamp = rospy.get_rostime()
        cloud_msg.header.frame_id = 'world'

        cloud_msg.poses = []

	filtro = Pose()
        filtro.position.x = self.ekf.x
        filtro.position.y = self.ekf.y
        filtro.position.z = self.ekf.z	

	filtro.orientation.x = 0.0
	filtro.orientation.y = 0.0
	filtro.orientation.z = 1.0
	filtro.orientation.w = 0.0

	cloud_msg.poses.append(filtro)

	#Posicion por pantalla
	#print(self.ekf.x, self.ekf.y ,self.ekf.z)

	for i in range(len(self.landmarks)):
		baliza = Pose()
	        baliza.position.x = self.landmarks[i][0]
	        baliza.position.y = self.landmarks[i][1]
	        baliza.position.z = self.landmarks[i][2]	
	
		baliza.orientation.x = 0.0
		baliza.orientation.y = 0.0
		baliza.orientation.z = 1.0
		baliza.orientation.w = 0.0

		cloud_msg.poses.append(baliza)			

	self.filter_publisher.publish(cloud_msg)
           

if __name__ == '__main__':
    try:
        rospy.init_node('ekf', anonymous=True)

        f = ekfilter()

        r = rospy.Rate(20) # 2hz

        while not rospy.is_shutdown():

            # If robot has moved update filter
            if f.check_robot_motion():
               
            	# Create sensor measurement
            	Z = f.sense()

            	# EKFilter steps
            	f.predict()

            	f.update(Z)
            	
            # Publish particles
            f.publish()

            r.sleep()

    except rospy.ROSInterruptException:
        pass
