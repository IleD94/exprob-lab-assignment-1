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

def make_ind_of_class_disjoint (class_name):
        """
        Disjoint all individuals of a class.
    
        Args:
            ind_name (str): individual to be added to the class.
            class_name (str): individual will be added to this class. It will be created a new class if it does not exist.
    
        Returns:
            bool: True if ontology is consistent, else False
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    
        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!
        """
        try:
            res = client.call('DISJOINT', 'IND', 'CLASS', [class_name])
    
        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
            
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
         #make_ind_of_class_disjoint ("PERSON")
         #make_ind_of_class_disjoint ("WEAPON")
         #make_ind_of_class_disjoint ("PLACE")
     
     def changes_and_apply(self):
         client.utils.apply_buffered_changes()
         time.sleep (3)
         client.utils.sync_buffered_reasoner()
         for x in suspects:
            if client.query.check_ind_exists(x):
               print ("Item", x,"is present")
         for x in weapons:
            if client.query.check_ind_exists(x):
               print ("Item", x,"is present")
         for x in rooms:
            if client.query.check_ind_exists(x):
               print ("Item", x,"is present")
               
     def hint_gen (self):
         global who, where, what, whoInC, whereInC, whatInC, whoInCons, whatInCons, whereInCons, whoInCons2,whatInCons2,whereInCons2
         self.who = list()
         self.where = list()
         self.what = list()
         self.whoInC = list()
         self.whereInC = list()
         self.whatInC = list()
         self.whoInCons = list()
         self.whatInCons = list()
         self.whereInCons = list()
         self.whatInCons2 = list()
         self.whoInCons2 = list()
         self.whereInCons2 = list()
         
         #case consistent hypos
         for IDcounter in range (5):# da 0 a 5
             self.who.append (random.choice(suspects))
             self.where.append (random.choice(rooms))
             self.what.append (random.choice(weapons))
         print (self.who)
         print (self.where)
         print (self.what)
         
         #case incomplete
         for IDcounter in range (2): #da 6 a 7
             self.whoInC.append (random.choice(suspects))
             self.whereInC.append (random.choice(rooms))
             self.whatInC.append (random.choice(weapons))
         print (self.whoInC)
         print (self.whereInC)
         print (self.whatInC)
         
         #case inconsistent
         for IDcounter in range (3): #da 8 a 10
             self.whoInCons.append (random.choice(suspects))
             self.whereInCons.append (random.choice(rooms))
             self.whatInCons.append (random.choice(weapons))
             self.whatInCons2.append (random.choice(weapons))
             self.whoInCons2.append (random.choice(suspects))
             self.whereInCons2.append (random.choice(rooms))
         print (self.whoInCons)
         print (self.whereInCons)
         print (self.whatInCons)
         
         return self.who, self.where, self.what, self.whoInC, self.whereInC, self.whatInC, self.whoInCons, self.whereInCons, self.whatInCons
             
             
     def hint_callback (self, req):
     # caso consistente
         ID = req.ID
         if ID < 5: #consistent
                myhintlist = [self.who[ID],self.where[ID],self.what[ID]]
                print (myhintlist)
                self.hint = random.choice (myhintlist)
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
         elif ID == 5:
                myhintlist = [self.whoInC[0],self.whereInC[0]]
                self.hint = random.choice (myhintlist)
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
         elif ID == 6:
                myhintlist = [self.whoInC[1],self.whatInC[1]]
                self.hint = random.choice (myhintlist)
                self.myID = '000'+str(ID)
                rospy.set_param ('HP'+str(ID), self.myID)
                print (self.hint, self.myID)
                return self.hint, self.myID
         else: #inconsistent
            for IDcount in range (0,3): ################### Rivedere questa parte, FA CAGARE!
               if ID >= 7:
                    myhintlist1 = [self.whoInCons[ID-7],self.whereInCons[ID-7],self.whatInCons[ID-7],self.whatInCons2[ID-7]]
                    print (myhintlist1)
                    myhintlist2 = [self.whoInCons[ID-7],self.whereInCons[ID-7],self.whatInCons[ID-7],self.whatInCons2[ID-7], self.whoInCons2 [ID-7], self.whereInCons2 [ID-7]]
                    print (myhintlist2)
                    ListHint = random.choice ([myhintlist1, myhintlist2])
                    print (ListHint)
                    self.hint = random.choice (ListHint)
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
    time.sleep (5)    
    settings.hint_gen ()     
    rospy.spin()
