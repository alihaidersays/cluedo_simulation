#! /usr/bin/env python

# import ros stuff
import rospy

from exprob_a1.srv import Start, StartResponse

# service callback

def main():
    rospy.init_node('user_interface')
    
    user_input = int(input("Enter 1 to start the game or 0 to quit: "))
    
    while not rospy.is_shutdown():
    	if (user_input == 1):
    		rospy.wait_for_service('/start_game')
    		try:
    			game_client = rospy.ServiceProxy('/start_game', Start)
    			game_client_response = game_client("Begin")

    			if(game_client_response.res == "Finished"):
    				print("Game complete!")
    				user_input = int(input("Enter 1 to restart the game or 0 to quit: "))

    		except rospy.ServiceException as e:
    			print("Service call failed: %s"%e)
    		
    		
    	
    	elif (user_input == 0):
    		break
    	
    	else:
    		print("Incorrect input!")
    		user_input = int(input("Enter 1 to start the game or 0 to quit: "))
    		
    	

if __name__ == '__main__':
    main()
