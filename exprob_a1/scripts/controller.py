#! /usr/bin/env python

# import ros stuff
import rospy
import time
from geometry_msgs.msg import Point

from exprob_a1.srv import Start, StartResponse, Hint, HintResponse, Oracle, OracleResponse

hint_client_ = None
oracle_client_ = None
ID_ = 0
flag_ = 0

# service callback

def start_game_callback(message):
	print("Message request:", message.req)
	
	if (message.req == "Begin"):
		if(runGame()):
			time.sleep(1)
			return StartResponse("Finished.")
	else:
		print("Request to start game failed!")
	
def movement(robot_x, robot_y, target_x, target_y):
	distance = (((target_x - robot_x )**2) + ((target_y - robot_y)**2) )**0.5

	if (target_y-robot_y) > 0:
		factor_y = 1
	elif (target_y-robot_y) < 0:
		factor_y = -1
	else:
		factor_y = 0
		
	if (target_x-robot_x) > 0:
		factor_x = 1
	elif (target_x-robot_x) < 0:
		factor_x = -1
	else:
		factor_x = 0
		
	step_x = 0.2 * factor_x
	step_y = 0.2 * factor_y
	
	while not distance == 0.0:
		print("Current position: (%g,%g); Distance remaining: %g" % (robot_x, robot_y, distance))
		robot_x = robot_x + step_x
		robot_y = robot_y + step_y
		distance = (((target_x - robot_x )**2) + ((target_y - robot_y)**2) )**0.5
		time.sleep(0.5)
	
	if (robot_x, robot_y == target_x, target_y):
		return True
			
	
def runGame():
	global ID_
	# initializing environment
	robot = Point()
	roomA = Point()
	roomB = Point()
	roomC = Point()
	checkpoint = Point()
	
	# setting positions
	robot.x, robot.y = 0.0, 0.0
	roomA.x, roomA.y = 1.0, 0.0
	roomB.x, roomB.y = 1.0, 1.0
	roomC.x, roomC.y = 0.0, 1.0
	checkpoint.x,checkpoint.y = 2.0, 2.0
	
	while True:
		# move to A
		if (movement(robot.x, robot.y, roomA.x, roomA.y)):
			print ("\nRobot reached Room A\n")
			# ask for hint for Room A
			hint_client_response = hint_client_("ID"+str(ID_))
			print("Hint Client Response: ", hint_client_response.res)
			time.sleep(1)
			# move to B
			if (movement(robot.x, robot.y, roomB.x, roomB.y)):
				print ("\nRobot reached Room B\n")
				# ask for hint for Room B
				hint_client_response = hint_client_("ID"+str(ID_))
				print("Hint Client Response: ", hint_client_response.res)
				time.sleep(1)
				# move to C
				if (movement(robot.x, robot.y, roomC.x, roomC.y)):
					print ("\nRobot reached Room C\n")
					# ask for hint for Room C
					hint_client_response = hint_client_("ID"+str(ID_))
					print("Hint Client Response: ", hint_client_response.res)
					time.sleep(1)
				else:
					print("Robot did not reach Room C")
			else:
				print("Robot did not reach Room B")
		else:
			print("Robot did not reach Room A")
		
		# start reasoner
		oracle_client_response = oracle_client_("Start reasoner")
		time.sleep(0.5)
		if oracle_client_response.res == "Reasoner started":
			print("\nReasoner started\n")
			oracle_client_response = oracle_client_("Check consistency")
			time.sleep(1)
			if oracle_client_response.res == "Hypothesis is consistent":
				# go to checkpoint
				if (movement(robot.x, robot.y, checkpoint.x, checkpoint.y)):
					print ("\nRobot reached the checkpoint\n")
					oracle_client_response = oracle_client_("Check correctness")
					time.sleep(1)
					if oracle_client_response.res == "Hypothesis is correct":
						ID_ = ID_ + 1
						print("\nThe hypothesis is correct. You win the game!\n")
						return True
					else:
						print("\nHypothesis is INCORRECT. Searching for another hypothesis!\n")
				else:
					print("\nHypothesis is INCONSISTENT. Searching for another hypothesis!\n")
		else:
			print("Oracle Client Response: ", oracle_client_response.res)

		ID_ = ID_ + 1

def main():
	global hint_client_
	global oracle_client_
	global flag_
	rospy.init_node('controller')
	
	if flag_ == 0:
		print("\nController node initialized.\n")
		flag_ = 1
	
	serv = rospy.Service('/start_game', Start, start_game_callback)
	
	rospy.wait_for_service('/hint')
	try:
		hint_client_ = rospy.ServiceProxy('/hint', Hint)
		
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
		
	rospy.wait_for_service('/oracle')
	try:
		oracle_client_ = rospy.ServiceProxy('/oracle', Oracle)
		
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
	
		
	rospy.spin()

if __name__ == '__main__':
    main()
