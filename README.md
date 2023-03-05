# exprob_ass1

![image](https://user-images.githubusercontent.com/80365922/222982473-5d452e0b-6312-4ff6-943e-64a08d113445.png)


## Introduction
This project is the first part out of three of the implementation of Cluedo Game in simulation environment. 
In this part we have implemented the architecture of the game, without any real movement of the player, a robot Detective.
In the architecture we have 6 components: Ontology, Armor service, Default Settings, Cluedo FSM, Oracle e User Interface.
The rules of the games are simple. There was an omicide, Mr. Black was killed by someone, somewhere with some weapon. There is a detective, Detective Bot, that goes around (not implemented yet in this part) to find hints to solve the case!

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

![image](https://user-images.githubusercontent.com/80365922/222924326-ebe58210-de53-43b9-8ea9-7f1d9775ff43.png)


### Cluedo FSM
Thid node implements a final state machine using smach package for ros. The states are: Exploration, Query and Oracle.
1. Exploration: simulates, using sleep of some seconds, the moving from a room to another of the robot. In this state we look for hints and collect them.
2. Query: asks some information about the hypothesis of the previous collected hint. Here we can understand if our hypothesis is consistent, incomplete or inconsistent. If the hypothesis is complete we can go to the next state, otherwise we need to go around searching for some more hints in the previous state.
3. Oracle: stores the winning hypothesis of the game. In particular a service HypothesisID stores the winning ID of the game. In this state we compare our hypothesis to the winning one to understand if we won or if we have to go back to the first state and continue the quest.

![image](https://user-images.githubusercontent.com/80365922/222924204-58142120-ba6c-41d8-aaef-447291a95d68.png)

### Oracle
Here it is implemented a service that waits for the request of the cluedo state machine, from the oracle state, like in the picture below, with the winning ID, it compares the two ids and return a boolean value. True if the hypothesis is correct and the player wins the game, false if not.

![image](https://user-images.githubusercontent.com/80365922/222924362-a67aaa75-9fc3-42e3-90b2-49c95e209e2f.png)
## Component diagram
![assignment1 drawio (1)](https://user-images.githubusercontent.com/80365922/222924085-c6add65b-8e76-40ae-b171-f53fa523d7b5.png)

## Sequence diagram

![assignment1seq drawio](https://user-images.githubusercontent.com/80365922/222924092-ca26d96e-2993-4196-9dea-55d8d132c31f.png)

## How to run
To install this package, please follow the command below:
```
git clone https://github.com/IleD94/exprob_ass_1
```

In order to run the code you have to launch the launch file present in the launch in cluedo folder.

```
roslaunch cluedo cluedo.launch
```
## Video



https://user-images.githubusercontent.com/80365922/222978412-8f933d34-c551-40d4-b8d5-67efe5cbe609.mp4

## System's limitations
A possible limitation in the system is in the generation of random hypothesis. The game can be very long, considering that we can have 10 hypothesis and that the inconsistent can have potentially infinite values. This make the game really long.

## Possible improvements
To improve the system we could have a maximum number of items for every inconsistent hypothesis

## Contacts

Author: Ilenia D'Angelo,

email: ileniadangelo94@gmail.com
