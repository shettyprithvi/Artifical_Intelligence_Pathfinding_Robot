# Artifical_Intelligent_Pathfinding_Robot

## Solution: To see the output, run main.py
The shortest path is found by the AI Robot by finding the best and the shortest path using A-star algorithm. The heuristic is the Euclidean distance between the robot and destination.

## Problem:

![alt text](https://github.com/shettyprithvi/Artifical_Intelligence_Pathfinding_Robot/blob/master/ai.png)

The challenge is to program an A* (A-star) heuristic search (graph search, Rich/Knight/Tanimoto version)
to allow an agent to compute the shortest path from a start point to a goal point that goes around obstacles.
The scenario of this problem is that the agent is located at point A and he needs to get to point C. 
He can move to any of a finite set of points in his environment. However, there are also a set of obstacles, 
so he cannot move directly from A to C, but must instead avoid the obstacles. 
The points in the space have real coordinates (x,y) and are just the vertices of the obstacles.

The agent wants to plan a path from A to C that does not cut across any of the obstacles.
A move must be from some vertex I to another vertex J (the robot will never be at a point other than a vertex, 
unless it is moving from one vertext to another). I and J may be on the same rectangle or on two different ones.

He is allowed to move along an edge of an obstacle, just not through it.

The possible operators at each state are just the moves to any of the other states. Many of them will be illegal, 
because the line from the current state to the other state intersects a rectangle. 

Therefore, you will need to code a utility function that determines if a given line segment intersects a given rectangle. 

The heuristic function h should use the straight line distance from the current vertex to the goal vertex, 
which can never overestimate the true distance. 
