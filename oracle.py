#!/usr/bin/env python

import rospy
import random
import time
from cluedo.srv import HypothesisID
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient


ID = ['0000','0001','0002','0003','0004'] #only first 6 can be consistent and the winning sequence

class oracle_server:

     def __init__(self):
         self.oracle_service = rospy.Service('compare_hypothesis', HypothesisID, self.comparison_callback)
         print("Tell me your hypothesis!")

     def random_winning_ID(self):
         global WinningID
         self.WinningID = random.choice (ID)
         print ('The winning ID is:', self.WinningID)
         return self.WinningID 
     
     def comparison_callback (self, req):
         print ('Your request is:', req.ID)
         if (req.ID == self.WinningID):
            return True
         else:
            return False

if __name__ == "__main__":
    rospy.init_node('compare_hypothesis')
    oracle = oracle_server()
    oracle.random_winning_ID()
    rospy.spin()
