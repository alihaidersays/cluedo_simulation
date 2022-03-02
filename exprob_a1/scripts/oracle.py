#! /usr/bin/env python2

# import ros stuff
import rospy
import time
from geometry_msgs.msg import Point
import random

from exprob_a1.srv import Start, StartResponse, Hint, HintResponse, Oracle, OracleResponse
from armor_msgs.srv import ArmorDirective, ArmorDirectiveRequest

armor_interface_client_ = None
flag_ = 0
counter_ = 0
hint_ = []
loaded_hints_ = []


correct_hyp = [["Rev. Green", "Candlestick", "Conservatory"], 
			   ["Rev. Green", "Dagger", "Conservatory"], 
			   ["Rev. Green", "Lead Pipe", "Conservatory"], 
			   ["Rev. Green", "Revolver", "Conservatory"], 
			   ["Rev. Green", "Rope", "Conservatory"], 
			   ["Rev. Green", "Spanner", "Conservatory"],
			   
			   ["Prof. Plum", "Candlestick", "Conservatory"], 
			   ["Prof. Plum", "Dagger", "Conservatory"], 
			   ["Prof. Plum", "Lead Pipe", "Conservatory"], 
			   ["Prof. Plum", "Revolver", "Conservatory"], 
			   ["Prof. Plum", "Rope", "Conservatory"], 
			   ["Prof. Plum", "Spanner", "Conservatory"],
			   
			   ["Col. Mustard", "Candlestick", "Conservatory"], 
			   ["Col. Mustard", "Dagger", "Conservatory"], 
			   ["Col. Mustard", "Lead Pipe", "Conservatory"], 
			   ["Col. Mustard", "Revolver", "Conservatory"], 
			   ["Col. Mustard", "Rope", "Conservatory"], 
			   ["Col. Mustard", "Spanner", "Conservatory"],
			   
			   ["Rev. Green", "Candlestick", "Lounge"], 
			   ["Rev. Green", "Dagger", "Lounge"], 
			   ["Rev. Green", "Lead Pipe", "Lounge"], 
			   ["Rev. Green", "Revolver", "Lounge"], 
			   ["Rev. Green", "Rope", "Lounge"], 
			   ["Rev. Green", "Spanner", "Lounge"],
			   
			   ["Prof. Plum", "Candlestick", "Lounge"], 
			   ["Prof. Plum", "Dagger", "Lounge"], 
			   ["Prof. Plum", "Lead Pipe", "Lounge"], 
			   ["Prof. Plum", "Revolver", "Lounge"], 
			   ["Prof. Plum", "Rope", "Lounge"], 
			   ["Prof. Plum", "Spanner", "Lounge"],
			   
			   ["Col. Mustard", "Candlestick", "Lounge"], 
			   ["Col. Mustard", "Dagger", "Lounge"], 
			   ["Col. Mustard", "Lead Pipe", "Lounge"], 
			   ["Col. Mustard", "Revolver", "Lounge"], 
			   ["Col. Mustard", "Rope", "Lounge"], 
			   ["Col. Mustard", "Spanner", "Lounge"],
			   
			   ["Rev. Green", "Candlestick", "Kitchen"],
			   ["Rev. Green", "Dagger", "Kitchen"], 
			   ["Rev. Green", "Lead Pipe", "Kitchen"], 
			   ["Rev. Green", "Revolver", "Kitchen"], 
			   ["Rev. Green", "Rope", "Kitchen"], 
			   ["Rev. Green", "Spanner", "Kitchen"],
			   
			   ["Prof. Plum", "Candlestick", "Kitchen"], 
			   ["Prof. Plum", "Dagger", "Kitchen"], 
			   ["Prof. Plum", "Lead Pipe", "Kitchen"], 
			   ["Prof. Plum", "Revolver", "Kitchen"], 
			   ["Prof. Plum", "Rope", "Kitchen"], 
			   ["Prof. Plum", "Spanner", "Kitchen"],
			   
			   ["Col. Mustard", "Candlestick", "Kitchen"], 
			   ["Col. Mustard", "Dagger", "Kitchen"], 
			   ["Col. Mustard", "Lead Pipe", "Kitchen"], 
			   ["Col. Mustard", "Revolver", "Kitchen"], 
			   ["Col. Mustard", "Rope", "Kitchen"], 
			   ["Col. Mustard", "Spanner", "Kitchen"]]


# service callback

def hint_callback(ID):
	global hint_
	hint_who = ["Rev. Green", "Prof. Plum", "Col. Mustard"] #Mrs. Peacock
	hint_what = ["Candlestick", "Dagger", "Lead Pipe", "Revolver", "Rope", "Spanner"]
	hint_where = ["Conservatory", "Lounge", "Kitchen"]
	hint_type = "" 
	x = random.choice([hint_who, hint_what, hint_where])

	if x == hint_who:
		hint_type = "who"
	elif x == hint_what:
		hint_type = "what"
	elif x == hint_where:
		hint_type = "where"
	
	generated_hint = random.choice(x)
	
	#print("ID request:", ID.req)
	
	if (ID.req != ""):
		hint_ = [hint_type, ID.req, generated_hint]
		if(loadHint(hint_)):
			#print("\nHint generated and loaded in the reasoner successfully.")
			return HintResponse(hint_)
		else:
			print("\nERROR: Hint did not load.")
	else:
		print("\nID request is incorrect!")

def oracle_callback(message):
	global counter_, loaded_hints_
	
	if (message.req == "Start reasoner"):
		oracle_req = ArmorDirectiveRequest()
		
		oracle_req.armor_request.client_name = 'tutorial'
		oracle_req.armor_request.reference_name = 'ontoTest'
		oracle_req.armor_request.command = 'REASON'
		oracle_req.armor_request.primary_command_spec = ''
		oracle_req.armor_request.secondary_command_spec = ''
		oracle_req.armor_request.args = []
	
		oracle_response = armor_interface_client_(oracle_req)
		
		if (oracle_response.armor_response.success == True):
			return OracleResponse("Reasoner started")
		else:
			return OracleResponse("Reasoner not started")

	elif (message.req == "Check consistency"):
		oracle_req = ArmorDirectiveRequest()
		
		oracle_req.armor_request.client_name = 'tutorial'
		oracle_req.armor_request.reference_name = 'ontoTest'
		oracle_req.armor_request.command = 'QUERY'
		oracle_req.armor_request.primary_command_spec = 'IND'
		oracle_req.armor_request.secondary_command_spec = 'CLASS'
		oracle_req.armor_request.args = ['COMPLETED']
	
		oracle_response = armor_interface_client_(oracle_req)
		
		if (oracle_response.armor_response.success == True):
			print("\nOracle communication established.\n")
			
			# checking consistency
			len_queried_objects = len(oracle_response.armor_response.queried_objects)
			if len_queried_objects > counter_:
				counter_ = len_queried_objects
				return OracleResponse("Hypothesis is consistent")
			else:
				loaded_hints_ = []
				return OracleResponse("Hypothesis is inconsistent")
		else:
			return OracleResponse("Request for checking consistency failed!")	
		
	elif (message.req == "Check correctness"):
		for i in correct_hyp:
			if (set(loaded_hints_) == set(i)):
				loaded_hints_ = i
				return OracleResponse("Hypothesis is correct")
		
		print("loaded_hints_: ", loaded_hints_)
		print("i: ", i)
		return OracleResponse("Hypothesis is incorrect")
		
	elif (message.req == "Send correct hints"):
		sentence = str(loaded_hints_[0]) + " with the " + str(loaded_hints_[1]) + " in the " + str(loaded_hints_[2])
		return OracleResponse(sentence)
	


def loadHint(hint_arg):
	global loaded_hints_
	print("Hint loaded: ", hint_arg)

	loaded_hints_.append(hint_arg[2])
	
	if hint_arg[0] == "who":
		hint_req = ArmorDirectiveRequest()
		hint_req.armor_request.client_name = 'tutorial'
		hint_req.armor_request.reference_name = 'ontoTest'
		hint_req.armor_request.command = 'ADD'
		hint_req.armor_request.primary_command_spec = 'OBJECTPROP'
		hint_req.armor_request.secondary_command_spec = 'IND'
		hint_req.armor_request.args = hint_arg
	
		hint_response = armor_interface_client_(hint_req)
	
		if (hint_response.armor_response.success == True):
			hint_class_req = ArmorDirectiveRequest()
			hint_class_req.armor_request.client_name = 'tutorial'
			hint_class_req.armor_request.reference_name = 'ontoTest'
			hint_class_req.armor_request.command = 'ADD'
			hint_class_req.armor_request.primary_command_spec = 'IND'
			hint_class_req.armor_request.secondary_command_spec = 'CLASS'
			hint_class_req.armor_request.args = [hint_arg[2],'PERSON']
			
			hint_class_res = armor_interface_client_(hint_class_req)
			
			if (hint_class_res.armor_response.success == True):
				return True
		
	elif hint_arg[0] == "what":
		hint_req = ArmorDirectiveRequest()
		hint_req.armor_request.client_name = 'tutorial'
		hint_req.armor_request.reference_name = 'ontoTest'
		hint_req.armor_request.command = 'ADD'
		hint_req.armor_request.primary_command_spec = 'OBJECTPROP'
		hint_req.armor_request.secondary_command_spec = 'IND'
		hint_req.armor_request.args = hint_arg
	
		hint_response = armor_interface_client_(hint_req)
	
		if (hint_response.armor_response.success == True):
			hint_class_req = ArmorDirectiveRequest()
			hint_class_req.armor_request.client_name = 'tutorial'
			hint_class_req.armor_request.reference_name = 'ontoTest'
			hint_class_req.armor_request.command = 'ADD'
			hint_class_req.armor_request.primary_command_spec = 'IND'
			hint_class_req.armor_request.secondary_command_spec = 'CLASS'
			hint_class_req.armor_request.args = [hint_arg[2],'WEAPON']
			
			hint_class_res = armor_interface_client_(hint_class_req)
			
			if (hint_class_res.armor_response.success == True):
				return True
				
	elif hint_arg[0] == "where":
		hint_req = ArmorDirectiveRequest()
		hint_req.armor_request.client_name = 'tutorial'
		hint_req.armor_request.reference_name = 'ontoTest'
		hint_req.armor_request.command = 'ADD'
		hint_req.armor_request.primary_command_spec = 'OBJECTPROP'
		hint_req.armor_request.secondary_command_spec = 'IND'
		hint_req.armor_request.args = hint_arg
	
		hint_response = armor_interface_client_(hint_req)
		
		if (hint_response.armor_response.success == True):
			hint_class_req = ArmorDirectiveRequest()
			hint_class_req.armor_request.client_name = 'tutorial'
			hint_class_req.armor_request.reference_name = 'ontoTest'
			hint_class_req.armor_request.command = 'ADD'
			hint_class_req.armor_request.primary_command_spec = 'IND'
			hint_class_req.armor_request.secondary_command_spec = 'CLASS'
			hint_class_req.armor_request.args = [hint_arg[2],'PLACE']
			
			hint_class_res = armor_interface_client_(hint_class_req)
			
			if (hint_class_res.armor_response.success == True):
				return True
	return False
				
def initializeReasoner():
	armor_req = ArmorDirectiveRequest()

	armor_req.armor_request.client_name = 'tutorial'
	armor_req.armor_request.reference_name = 'ontoTest'
	armor_req.armor_request.command = 'LOAD'
	armor_req.armor_request.primary_command_spec = 'FILE'
	armor_req.armor_request.secondary_command_spec = ''
	armor_req.armor_request.args = ['/root/Desktop/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
	
	armor_interface_client_response = armor_interface_client_(armor_req)
	if (armor_interface_client_response.armor_response.success == True):
		print("\nReasoner initialized.\n")


def main():
	global armor_interface_client_
	global flag_
	rospy.init_node('oracle')

	serv = rospy.Service('/hint', Hint, hint_callback)
	
	serv2 = rospy.Service('/oracle', Oracle, oracle_callback)
	
	rospy.wait_for_service('/armor_interface_srv')
	try:
		armor_interface_client_ = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective)
		
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
	
	if flag_ == 0:
		initializeReasoner()
		flag_ = 1
	
	rospy.spin()

if __name__ == '__main__':
    main()
