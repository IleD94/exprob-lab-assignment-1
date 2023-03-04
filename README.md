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

## Implementation
