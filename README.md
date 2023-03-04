# exprob_ass1
## Introduction
This project is the first part out of three of the implementation of Cluedo Game in simulation environment. 
In this part we have implemented the architecture of the game, without any real movement of the player, a robot Detective.
The architecture is composed of 6 components: Ontology, Armor service, Default Settings, Cluedo FSM, Oracle e User Interface.
The rules of the games are simple. There was a omicide, Mr. Black was killed by someone, somewhere with some weapone. There is a detective, Detective Bot, that goes around (not implemented yet in this part) to find hints to solve the case!

## Suspects
Mr. Black was killed by some one. The killer is hiddin among these suspects:
1. Miss Scarlett
2. Mrs White
3. Mrs Peacock
4. Reverend Green
5. Professor Plum
6. Colonel Mustard

## Weapons
The weapon used to kill Mr. Black can be one of these:
1. Dagger
2. Candle Stick
3. Revolver
4. Rope
5. Lead Pipe
6. Spanner

## Rooms
We don't know also the place of the murder. It's one among the rooms of the house:
1. Hall
2. Lounge
3. Dining Room
4. Kitchen
5. Ball Room
6. Conservatory
7. Billiard Room
8. Library
9. Study

## Hints
Hints are be found going around, in each room. Every hint contain an ID, the source that gave us the hint, a key: what, where, who, that is directly understandable from its value, one among the suspects if it is 'who', one among weapons for 'what' and one among rooms for 'where'. In this implementation hints are produced by the Default Settings component. It manages the addition of suspects, rooms, weapons at the begining of the game and each hypothesis, during the exploration, to the ontology.
Every ID creates a hypothesis. Every hypothesis can be: consistent, inconsistent, incomplete.
To be consistent a hypothesis has to have a value and no more for each key.
To be inconsistent a hypothesis has to have at least one of the key field filled by more than one value.
To be incompleted a hypothes has to have at least one of the key filled void.

In order to check the state of each hypothesis we uses the query of armor services to ask to the ontology of the game.
## Implementation
In this section we'll go deeper in each component of the architecture.

### Ontology
The ontology of the game is managed by armor service. In the implementation we used armor_api (implemented in armor package, property of emarolab). Here are present at the end of the game every information about the hypothesis generated and found during the quest.
### Armor
This service was implemented by EmaroLab. To have more info about that goes to: (AGGIUNGERE LINK)
### Default Settings
In this node we implemented the default settings of the game, adding classes: person, rooms and weapons to the ontology. Moreover here it is implemented the generation of hints. That are comunicated using the Hint service, to have more info about the implementation of the service, please go to the doxygen documentation of this repository, in the cluedo folder.
### User interface
This node is used to show, printing on the display some messages, to the user what is happening in the game. To do that a String message is used to be published to the proper topic. For more info, please, refer to doxygen documentation in the cluedo folder of this repository.
### Cluedo FSM
Thid node implements a final state machine using smach package for ros. The states are: Exploration, Query and Oracle.
1. Exploration: simulates, using sleep of some seconds, the moving from a room to another of the robot. In this state we look for hints and collect them.
2. Query: asks some information about the hypothesis of the previous collected hint. Here we can understand if our hypothesis is consistent, incomplete or inconsistent. If the hypothesis is complete we can go to the next state, otherwise we need to go around searching for some more hints in the previous state.
3. Oracle: stores the winning hypothesis of the game. In particular a service HypothesisID stores the winning ID of the game. In this state we compare our hypothesis to the winning one to understand if we won or if we have to go back to the first state and continue the quest.
(AGGIUNGERE IMMAGINE)

## Component diagram
(AGGIUNGERE COMPONENT DIAGRAM CON QUALCHE SPIEGAZIONE)

## Sequence diagram
(AGGIUNGERE SEQUENCE DIAGRAM CON QUALCHE SPIEGAZIONE)

## How to run

In order to run the code you have to launch the launch file present in the launch in cluedo folder.

'roslaunch cluedo cluedo.launch'

## Contacts

Autor: Ilenia D'Angelo
email: ileniadangelo94@gmail.com
