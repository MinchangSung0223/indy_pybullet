import pybullet as p
import time
import numpy as np
import pybullet_data
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.3f}".format(x)})
import math
pi = math.pi
import cv2
jointPoses = [0,0,0,0,0,0,0]
ll = [0,-175/180*pi,-175/180*pi,-175/180*pi,-175/180*pi,-175/180*pi,-215/180*pi,0]
ul = [0,175/180*pi,175/180*pi,175/180*pi,175/180*pi,175/180*pi,215/180*pi,0]
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1,0.1]
jr = [0, 6.10865, 6.10865, 6.10865, 6.10865, 6.10865, 6.10865]
rp = [0,-0.30*pi/180,-14.98*pi/180,-89.91*pi/180,-0.0*pi/180,-75.48*pi/180,0,0]
indyEndEffectorIndex = 7
imageWidth  = 640;
imageHeight  = 640;
gripper_state = 0
gripperPoses = [0,0,0,0]
gripper_open = [0,0,0,0]
gripper_close = [pi+pi/4,pi,pi+pi/4,pi]

def getCameraImageEEF(eef_pose):
	#print(eef_pose)
	com_p = eef_pose[0]
	com_o = eef_pose[1]
	rot_matrix = p.getMatrixFromQuaternion(com_o)
	rot_matrix = np.array(rot_matrix).reshape(3, 3)
	# Initial vectors
	init_camera_vector = (0, 0, 1) # z-axis
	init_up_vector = (0, 1, 0) # y-axis
	# Rotated vectors
	camera_vector = rot_matrix.dot(init_camera_vector)
	up_vector = rot_matrix.dot(init_up_vector)
	view_matrix = p.computeViewMatrix([com_p[0],com_p[1],com_p[2]],[com_p[0],com_p[1],com_p[2]] + 100 * camera_vector, up_vector)
	fov = 60
	aspect = imageWidth/imageHeight
	near = 0.01
	far = 1000
	angle = 0.0;


	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(imageWidth,
					imageHeight,
					view_matrix,
					projection_matrix,
					shadow=True,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images
def getMatrixfromEEf(eef_pose):
	Tbe = np.eye(4)
	eef_p = np.array(eef_pose[0])
	eef_o = np.array(eef_pose[1])
	#print(eef_o)
	R = np.reshape(p.getMatrixFromQuaternion(eef_o),[3,3])
	R2 = []
	Tbe[0:3,0:3] = R
	Tbe[0,3] = eef_p[0]
	Tbe[1,3] = eef_p[1]
	Tbe[2,3] = eef_p[2]

	return Tbe
def printHomogeneous(T,print_str):


	print_str = " ------------------"+print_str+"------------------"
	print(print_str)
	#print(T)
	float_point = 3
	print("|\t"+"   "+"\t"+"  R"+"\t"+"   "+"\t\t|")
	print("|\t"+str("{:.3f}".format(np.around(T[0,0],float_point)))+"\t"+str("{:.3f}".format(np.around(T[0,1],float_point)))+"\t"+str("{:.3f}".format(np.around(T[0,2],float_point)))+"\t\t|")
	print("|\t"+str("{:.3f}".format(np.around(T[1,0],float_point)))+"\t"+str("{:.3f}".format(np.around(T[1,1],float_point)))+"\t"+str("{:.3f}".format(np.around(T[1,2],float_point)))+"\t\t|")
	print("|\t"+str("{:.3f}".format(np.around(T[2,0],float_point)))+"\t"+str("{:.3f}".format(np.around(T[2,1],float_point)))+"\t"+str("{:.3f}".format(np.around(T[2,2],float_point)))+"\t\t|")
	print("|\t"+"  x"+"\t"+"  y"+"\t"+"  z"+"\t\t|")
	print("|\t"+str("{:.3f}".format(np.around(T[0,3],float_point)))+"\t"+str("{:.3f}".format(np.around(T[1,3],float_point)))+"\t"+str("{:.3f}".format(np.around(T[2,3],float_point)))+"\t\t|")
	end_str=""
	for i in range(len(print_str)):
		end_str = end_str+"-"
	print(end_str)
	lineLen = 0.1
	pos = [T[0,3],T[1,3],T[2,3]]
	dir0 = [T[0,0],T[1,0],T[2,0]]
	dir1 = [T[0,1],T[1,1],T[2,1]]
	dir2 = [T[0,2],T[1,2],T[2,2]]

	toX = [pos[0] + lineLen * dir0[0], pos[1] + lineLen * dir0[1], pos[2] + lineLen * dir0[2]]
	toY = [pos[0] + lineLen * dir1[0], pos[1] + lineLen * dir1[1], pos[2] + lineLen * dir1[2]]
	toZ = [pos[0] + lineLen * dir2[0], pos[1] + lineLen * dir2[1], pos[2] + lineLen * dir2[2]]
	p.addUserDebugLine(pos, toX, [1, 0, 0], 2,0.1)
	p.addUserDebugLine(pos, toY, [0, 1, 0], 2,0.1)
	p.addUserDebugLine(pos, toZ, [0, 0, 1], 2,0.1)


if __name__ == "__main__":
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)
	p.setGravity(0, 0, -10)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.loadURDF("checkerboard/calibration.urdf", [0.5, 0, 0.01])
	p.loadURDF("plane.urdf", [0, 0, 0.0])
	indyId = p.loadURDF("indy7_robots_gripper/indy7_gripper.urdf", [0, 0, 0.05],p.getQuaternionFromEuler([0, 0, 0]))
	numJoints = p.getNumJoints(indyId) 
	for i in range(7):
		p.resetJointState(indyId, i, rp[i])
	t = 0
	while(1):
		t = t + 0.01
		pos = [0.4+0.05*math.sin(t*4),0, 0.5]
		orn = p.getQuaternionFromEuler([0, -math.pi, 0])

		jointPoses = p.calculateInverseKinematics(indyId, indyEndEffectorIndex, pos, orn, ll, ul,jr, rp)
		camera_pose = p.getLinkState(indyId,8)
		image =getCameraImageEEF(camera_pose) 
		eef_pose = p.getLinkState(indyId,7)
		Tbe = getMatrixfromEEf(eef_pose)
		Tec = np.array([[0 ,-1 ,0, 0.0],[1 ,0 ,0, 0.0],[0 ,0 ,1, 0.0],[0,0,0,1]])
		Tbc = np.matmul(Tbe,Tec)
		printHomogeneous(Tbe,"Tbe")
		#print(image)
		if t>100:
			gripper_state = 1
		for i in range(0,numJoints):
			jointInfo = p.getJointInfo(indyId, i)
			jointID = jointInfo[0]
			jointName = jointInfo[1].decode("utf-8")
			#print(i,"jointName : ",jointName)
			if i<=6:
				 p.setJointMotorControl2(bodyIndex=indyId,
		                        jointIndex=i+1,
		                        controlMode=p.POSITION_CONTROL,
		                        targetPosition=jointPoses[i],
		                        targetVelocity=0,
		                        force=500,
		                        positionGain=0.15,
		                        velocityGain=1.5)
			elif i>=10:
				if gripper_state ==0:
					gripperPoses = gripper_open;
				else:
					gripperPoses = gripper_close;
				p.setJointMotorControl2(bodyIndex=indyId,
		                        jointIndex=i,
		                        controlMode=p.POSITION_CONTROL,
		                        targetPosition=0.5,
		                        targetVelocity=gripperPoses[i-10],
		                        force=500,
		                        positionGain=0.03,
		                        velocityGain=1)
		p.stepSimulation()
