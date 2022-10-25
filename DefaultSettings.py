#!/usr/bin/env python

import rospy
import random
import time
from cluedo.srv import Hint
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient

suspects = ["Miss Scarlett","Mrs White","Mrs Peacock", "Reverend Green", "Professor Plum", "Colonel Mustard"]
weapons = ["Dagger", "Candle Stick", "Revolver", "Rope", "Lead Pipe", "Spanner"]
rooms = ["Hall", "Lounge","Dining Room", "Kitchen", "Ball Room", "Conservatory", "Billiard Room", "Library", "Study"]


class DefaultSettings:

     def __init__(self): 
         self.hint_service = rospy.Service('hint_generator', Hint, self.hint_callback)
         print("Ready to start the game")

     def load_ontology (self):
         global client
         client = ArmorClient("cluedo", "ontology")
         client.utils.load_ref_from_file("/root/Desktop/cluedo_ontology.owl", "http://www.emarolab.it/cluedo-ontology",
                                True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
         client.utils.mount_on_ref()
         client.utils.set_log_to_terminal(True)
     
     def add_item_into_class (self):
         for x in suspects:
            client.manipulation.add_ind_to_class(x, "PERSON")
            print ("Added", x,"to person")
         for x in weapons:
            client.manipulation.add_ind_to_class(x, "WEAPON")
            print ("Added", x, "WEAPON")
         for x in rooms:
            client.manipulation.add_ind_to_class(x, "PLACE")
            print ("Added", x, "to Place")

     
     def changes_and_apply(self):
         client.utils.apply_buffered_changes()
         time.sleep (3)
         client.utils.sync_buffered_reasoner()
         #for x in suspects:
          #  if client.query.check_ind_exists(x):
            #   print ("Item", x,"is present")
         #for x in weapons:
          #  if client.query.check_ind_exists(x):
             #  print ("Item", x,"is present")
         #for x in rooms:
            #if client.query.check_ind_exists(x):
          #     print ("Item", x,"is present")
               
     def hint_gen (self):
         global who, where, what
         ###################################################VEDERE MEGLIO LA QUESTIONE DEI SELF####################################################
         self.who = list()
         self.where = list()
         self.what = list() 
         
         #gen of random hints in a list of 12 possibilities
         for IDcounter in range (12):# da 0 a 5
             self.who.append (random.choice(suspects))
             self.where.append (random.choice(rooms))
             self.what.append (random.choice(weapons))
         return self.who, self.where, self.what
             
             
     def hint_callback (self, req):
         ID = req.ID
         if ID < 5: #consistent hypothesis
                myhintlist = [self.who[ID],self.where[ID],self.what[ID]]
                print (myhintlist)
                self.hint = random.choice (myhintlist)
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
         elif ID == 5: # incomplete hypothesis
                myhintlist = [self.who[ID],self.where[ID]]
                print (myhintlist)
                self.hint = random.choice (myhintlist)
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
         elif ID == 6: #incomplete hypothesis
                myhintlist = [self.who[ID],self.what[ID]]
                print (myhintlist)
                self.hint = random.choice (myhintlist)
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
         else: #possible inconsistent hypothesis
                myhintlist1 = [self.who[random.randint (0,11)], self.where[random.randint (0,11)],self.what[random.randint (0,11)]]
                print (myhintlist1)
                myhint = random.choice (myhintlist1)
                self.hint = myhint
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
                    
                    
if __name__ == "__main__":
    rospy.init_node('default_settings')
    settings = DefaultSettings ()
    settings.load_ontology()
    settings.add_item_into_class()
    settings.changes_and_apply()    
    #time.sleep (5)    
    settings.hint_gen ()     
    rospy.spin()
