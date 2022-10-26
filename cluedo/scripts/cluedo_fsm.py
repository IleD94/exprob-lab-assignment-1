#!/usr/bin/env python

""" 
@package cluedo state machine
This node handles the states of the FSM
divided in: EXPLORATION, QUERY, ORACLE
"""
import rospy
import smach
import smach_ros
import random
import time
from cluedo.srv import Hint, HypothesisID
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient
from std_msgs.msg import String

######Global declarations

suspects = ["Miss Scarlett","Mrs White","Mrs Peacock", "Reverend Green", "Professor Plum", "Colonel Mustard"]
weapons = ["Dagger", "Candle Stick", "Revolver", "Rope", "Lead Pipe", "Spanner"]
rooms = ["Hall", "Lounge","Dining Room", "Kitchen", "Ball Room", "Conservatory", "Billiard Room", "Library", "Study"]

client = ArmorClient("cluedo", "ontology")

hp = [[],[],[],[],[],[],[],[],[],[]]


def make_ind_of_class_disjoint (class_name):
        """
        /brief Disjoint all individuals of a class in an ontology by comunication with the armor server.
    
        Args:
            ind_name (str): individual to be disjointed to the class.
            class_name (str): individual will be disjointed to this class. It will be created a new class if it does not exist.
    
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
            
def user_interface(msg):

    
        pub = rospy.Publisher('cluedo_ui', String, queue_size=10) 
        time.sleep(1)
        try:
            rospy.loginfo(msg)
            pub.publish(msg)
        except rospy.ROSInterruptException:
            pass
        
        


class hint_gen:
      """
      Class hint_gen manages the client of the hint_generator service 
      custom service: Hint

      """
      def __init__(self):
          rospy.wait_for_service('hint_generator')
          self.srv_hint_client = rospy.ServiceProxy('hint_generator', Hint)
      
      def hint_client(self, ID):
          """
          /brief client of the serice hint_generator
          custom service: Hint
              uint32 ID
              ---
              string myID
              string hint
          @param req: uint32 ID
          @return string: myID a code for the source 
          @return string: hint an hint of kind who, where or what
          """
          global res
          try:  
              self.res = self.srv_hint_client (ID)
              user_interface ('Your hint is: '+ self.res.hint + ' and the ID is: '+ self.res.myID)
              return self.res
          except rospy.ServiceException as e:
              print("Service call failed: %s"%e)


class oracle_query:
      """
      Class oracle_query manages the client of the compare_hypothesis service 
      custom service: HypotesisID

      """
      def __init__(self):
          rospy.wait_for_service('compare_hypothesis')
          self.srv_oracle_client = rospy.ServiceProxy('compare_hypothesis', HypothesisID)
      
      def oracle_client(self, myID):
          """
          /brief client of the service compare_hypotesis
          custom service: HypothesisID
              uint32 ID
              ---
              bool success
          @param req: uint32 ID
          @return bool: true if the the elements are the same, false if they are not
          """    
          global oracle_res
          try:  
              self.oracle_res = self.srv_oracle_client (myID)
              return self.oracle_res
          except rospy.ServiceException as e:
              print("Service call failed: %s"%e)


def add_hypothesis (ID, item):
    """
    /brief this function add items in the class HYPOTHESIS in the cluedo ontology
    and add an individual to the object property who, where and what, depending on 
    the kind of hint that it has received from the hint_generator. At the end it
    saves the changes and synchronize the reasoner using armor api.
    @param ID: uint32 with the number of the source
    @param item: the hint to add in the cluedo ontology as where, who or what instance 
    """
    global HP
    
    HP = "HP"+str(ID)
    myID = rospy.get_param (HP) #CONSISTENT
    client.manipulation.add_ind_to_class(HP, "HYPOTHESIS")
    client.manipulation.add_dataprop_to_ind("hasID", "HP"+str(ID), "STRING", myID)
    if item in suspects:
        client.manipulation.add_objectprop_to_ind("who", HP, item)
    elif item in rooms:
        client.manipulation.add_objectprop_to_ind("where", HP, item)
    elif item in weapons:
        client.manipulation.add_objectprop_to_ind("what", HP, item)
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()
    return ()


def storage_hypo (ID, item, hpi):
    """
    /brief this function add the last hint to the 
    list of the hypothesis if it is not present yet
    @param req: uint32 ID
    @param item: the hint to add in the list of the specific hypothesis 
    @param hpi: the array of the hypothesis that contains the hints that we got until now from that source 
    """
    if item not in hpi:
       hpi.append (item) 
       print (hpi , 'The ID is:', ID) 
    return ()

 
def winning_sequence (myID):
    """
    This function recognizes and sorts into killer, weapon and room the hints of
    a complete and consistent hypothesis. In order to check if can be the winning
    sequence of the game
    @param my ID: string with the code of one source
    @return string killer, weapon and room
    """
    for i in range (10):
        if myID == '000'+str(i):
           for j in range(len(hp[i])):
               if hp[i][j] in suspects:
                  killer = hp[i][j]
               elif hp[i][j] in weapons:
                  weapon = hp[i][j]
               elif hp[i][j] in rooms:
                  room = hp[i][j]
               else: 
                  print ('Error in the winning sequence')
    return killer, weapon, room  


# define state1 EXPLORATION
class Exploration(smach.State):
    """
    Classe Exploration it is the first state of the cluedo fsm
    Here the robot chooses randomly rooms and reaches them, then it gets
    an hint and restart again this process until it has an hypothesis with 3 items,
    when this happens the state machines goes into the state 2 QUERY 
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['check_hypo'],
                             #input_keys=['foo_counter_in'],
                             output_keys=['myID'])

    def execute(self,userdata):
        rospy.loginfo('Executing state Exploration')
        global ID
        get_random_room = random.choice(rooms)
        user_interface ("I'm going to the: "+ get_random_room)
        time.sleep(5)
        user_interface ("REACHED!")
        ID = random.randint(0,9)
        hint = hint_gen ()
        res = hint.hint_client(ID)
        userdata.myID=res.myID
        add_hypothesis (ID, res.hint)
        for i in range (len(hp)):
            if HP == 'HP'+str(i):
               storage_hypo (ID, res.hint, hp[i])
               user_interface ('This is what we know until now about this hypothesis: '+ str(hp[ID]))
               return 'check_hypo'
        
# define state2 Query
class Query(smach.State):
    """
    Class Query is the second state of the cluedo fsm.
    Here the robot checks if the hypothesis is complete and not inconsistent, talking
    with the armor server using the api of the armor client,
    if it is in this way it goes to the third state ORACLE to make the accusation and
    check if it is the winning hypothesis, otherwise it returns to the state1
    """
    def __init__(self):
    
        smach.State.__init__(self, 
                             outcomes=['go_around', 'go_to_oracle']
                             )
        
    def execute(self, userdata):
        rospy.loginfo('Executing state QUERY')       
        inconsistent_list = client.query.ind_b2_class("INCONSISTENT")
        complete_list = client.query.ind_b2_class("COMPLETED")
        inconsistent_str = str (inconsistent_list)
        complete_str = str (complete_list)
        if (complete_str.find ("HP"+str(ID)) != -1):
           user_interface ('The HP'+str(ID)+' is COMPLETE')
           if (inconsistent_str.find ("HP"+str(ID)) == -1):
              user_interface ('The HP'+str(ID)+' is CONSISTENT')
              time.sleep(1)
              return 'go_to_oracle'
           else:
              user_interface ('The HP'+str(ID)+' is INCONSISTENT')
              time.sleep(1)
              return 'go_around'
        else:
           user_interface ('The HP'+str(ID)+' is INCOMPLETE')
           return 'go_around'
        
#define state3 Oracle
class Oracle (smach.State):
    """
    Class Oracle is the third state of the cluedo fsm.
    It checks if the hypothesis received has the winning ID code, asking
    to the oracle that compares it with the ID of the winner.
    If it is not the winner. It return to the state 1 otherwise it ends the
    state machine.
    """
    
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_around', 'end'],
                             input_keys=['myID'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Oracle')
        user_interface ("I'm going to the Oracle room...")
        time.sleep (5)
        user_interface ("REACHED!")
        [killer, weapon, room] = winning_sequence (userdata.myID)
        user_interface ('I accuse! It was '+ killer +' with '+ weapon +' in '+ room+ '!')
        oracle = oracle_query()
        res= oracle.oracle_client(userdata.myID)
        if res.success :
           user_interface ('Your hypotesis with ID: '+ userdata.myID + ' is: '+ str (res.success) + ' You WIN, Detective Bot!')
           user_interface (killer + ' killed Dr. Black with '+ weapon+ ' in the '+ room)   
           client.utils.save_ref_with_inferences("/root/Desktop/inferred_cluedo.owl")     
           return 'end' #end of the game
        else:
           user_interface ('Your hypotesis with ID: '+ userdata.myID + ' is: '+ str(res.success))
           user_interface ('You LOOSE, go on with your research, Detective Bot!')
           return 'go_around'                     


def main():
    rospy.init_node('cluedo_fsm')
    time.sleep (5)
    make_ind_of_class_disjoint ("PERSON")
    make_ind_of_class_disjoint ("WEAPON")
    make_ind_of_class_disjoint ("PLACE")
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = ''
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('EXPLORATION', Exploration(), 
                               transitions={'check_hypo':'QUERY'},
                               remapping={'myID':'sm_counter'})
                               
        smach.StateMachine.add('QUERY', Query(), 
                               transitions={'go_to_oracle':'ORACLE', 
                                            'go_around':'EXPLORATION'})
                               
        smach.StateMachine.add('ORACLE', Oracle(), 
                               transitions={'go_around':'EXPLORATION', 
                                            'end' : 'outcome4'},
                               remapping={'myID':'sm_counter'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    
    # Execute SMACH plan
    sm.execute()
    
    while not rospy.is_shutdown():
       rospy.spin()
       sis.stop()


if __name__ == '__main__':
    main()
