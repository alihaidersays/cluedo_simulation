#! /usr/bin/env python

# import ros stuff
import rospy
import time
from geometry_msgs.msg import Point
import random

from exprob_a1.srv import Start, StartResponse, Hint, HintResponse, Oracle, OracleResponse
from armor_msgs.srv import ArmorDirective, ArmorDirectiveRequest

armor_interface_client_ = None
flag_ = 0

# service callback

def hint_callback(ID):
	hint_who = ["Rev. Green", "Prof. Plum", "Col. Mustard", "Mrs. Peacock", "Miss. Scarlett", "Mrs. White"]
	hint_what = ["Candlestick", "Dagger", "Lead Pipe", "Revolver", "Rope", "Spanner"]
	hint_where = ["Conservatory", "Lounge", "Kitchen", "Library", "Hall", "Study", "Ballroom", "Dining Room", "Billiard Room"]
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
		hint = [hint_type, ID.req, generated_hint]
		if(loadHint(hint)):
			print("\nHint generated and loaded in the reasoner successfully.")
			print(hint)
			return HintResponse(hint)
		else:
			print("\nERROR: Hint did not load.")
	else:
		print("ID request is incorrect!")

def oracle_callback(message):
	print("Message request:", message.req)
	
	if (message.req == "Request Oracle"):
		return OracleResponse("Oracle")
	else:
		print("Message request is incorrect!")


def loadHint(hint):
	print("hint", hint)
	if hint[0] == "who":
		print("who")
		hint_req = ArmorDirectiveRequest()
		hint_req.armor_request.client_name = 'tutorial'
		hint_req.armor_request.reference_name = 'ontoTest'
		hint_req.armor_request.command = 'ADD'
		hint_req.armor_request.primary_command_spec = 'OBJECTPROP'
		hint_req.armor_request.secondary_command_spec = 'IND'
		hint_req.armor_request.args = hint
	
		hint_response = armor_interface_client_(hint_req)
	
		if (hint_response.armor_response.success == True):
			hint_class_req = ArmorDirectiveRequest()
			hint_class_req.armor_request.client_name = 'tutorial'
			hint_class_req.armor_request.reference_name = 'ontoTest'
			hint_class_req.armor_request.command = 'ADD'
			hint_class_req.armor_request.primary_command_spec = 'IND'
			hint_class_req.armor_request.secondary_command_spec = 'CLASS'
			hint_class_req.armor_request.args = [hint[2],'PERSON']
			
			hint_class_res = armor_interface_client_(hint_class_req)
			
			if (hint_class_res.armor_response.success == True):
				return True
		
	elif hint[0] == "what":
		print("what")
		hint_req = ArmorDirectiveRequest()
		hint_req.armor_request.client_name = 'tutorial'
		hint_req.armor_request.reference_name = 'ontoTest'
		hint_req.armor_request.command = 'ADD'
		hint_req.armor_request.primary_command_spec = 'OBJECTPROP'
		hint_req.armor_request.secondary_command_spec = 'IND'
		hint_req.armor_request.args = hint
	
		hint_response = armor_interface_client_(hint_req)
	
		if (hint_response.armor_response.success == True):
			hint_class_req = ArmorDirectiveRequest()
			hint_class_req.armor_request.client_name = 'tutorial'
			hint_class_req.armor_request.reference_name = 'ontoTest'
			hint_class_req.armor_request.command = 'ADD'
			hint_class_req.armor_request.primary_command_spec = 'IND'
			hint_class_req.armor_request.secondary_command_spec = 'CLASS'
			hint_class_req.armor_request.args = [hint[2],'WEAPON']
			
			hint_class_res = armor_interface_client_(hint_class_req)
			
			if (hint_class_res.armor_response.success == True):
				return True
				
	elif hint[0] == "where":
		print("where")
		hint_req = ArmorDirectiveRequest()
		hint_req.armor_request.client_name = 'tutorial'
		hint_req.armor_request.reference_name = 'ontoTest'
		hint_req.armor_request.command = 'ADD'
		hint_req.armor_request.primary_command_spec = 'OBJECTPROP'
		hint_req.armor_request.secondary_command_spec = 'IND'
		hint_req.armor_request.args = hint
	
		hint_response = armor_interface_client_(hint_req)
		
		if (hint_response.armor_response.success == True):
			hint_class_req = ArmorDirectiveRequest()
			hint_class_req.armor_request.client_name = 'tutorial'
			hint_class_req.armor_request.reference_name = 'ontoTest'
			hint_class_req.armor_request.command = 'ADD'
			hint_class_req.armor_request.primary_command_spec = 'IND'
			hint_class_req.armor_request.secondary_command_spec = 'CLASS'
			hint_class_req.armor_request.args = [hint[2],'PLACE']
			
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
		print("\nReasoner initialized.")

	"""
	rosservice call /armor_interface_srv "armor_request:
  client_name: 'tutorial'
  reference_name: 'ontoTest'
  command: 'LOAD'
  primary_command_spec: 'FILE'
  secondary_command_spec: ''
  args: ['/root/Desktop/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']"
	"""
	
def main():
	global armor_interface_client_
	global flag_
	rospy.init_node('oracle')

	serv = rospy.Service('/hint', Hint, hint_callback)
	
	serv2 = rospy.Service('/oracle', Oracle, oracle_callback)		# yet to complete
	
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
