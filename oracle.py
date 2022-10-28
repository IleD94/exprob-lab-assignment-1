#!/usr/bin/env python

""" 
@package cluedo oracle
This node is the server of the service compare_hypotesis
it sets the winning ID of the game and compare this with the ID
of the accusation hypothesis
"""

import rospy
import random
import time
from cluedo.srv import HypothesisID
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient


ID = ['0000','0001','0002','0003','0004'] #only first 6 can be consistent and the winning sequence

class oracle_server:

     """
     Class of the oracle_server it manages the server of the 
     compare_hypothesis service. It has one function sets randomly the winning ID of
     the game and the callback of the service that 
     compares that with the ID of the accusation hypothesis
     """
     def __init__(self):
         self.oracle_service = rospy.Service('compare_hypothesis', HypothesisID, self.comparison_callback)
         print("Tell me your hypothesis!")

     def random_winning_ID(self):
         """
         /brief Randomly it chooses one ID that 
         will be the winning one of the game
         """
         global WinningID
         self.WinningID = random.choice (ID)
         print ('The winning ID is:', self.WinningID)
         return self.WinningID 
     
     def comparison_callback (self, req):
         """
         /brief it is the callback of the compare hypothesis service 
         compare one ID that will be the winning one of the game
         custom service: HypothesisID
              uint32 ID
              ---
              bool success
         @param req: uint32 ID
         @return bool: true if the the elements are the same, false if they are not
         """
         print ('Your request is:', req.ID)
         if (req.ID == self.WinningID):
            return True
         else:
            return False

if __name__ == "__main__":
    rospy.init_node('oracle')
    oracle = oracle_server()
    oracle.random_winning_ID()
    while not rospy.is_shutdown():
       rospy.spin()
