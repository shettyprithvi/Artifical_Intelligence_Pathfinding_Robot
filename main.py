#!/usr/bin/env python

#Prithvi Shetty
#Assignment 2
#Code written in Python 2



import numpy as np
import warnings
warnings.filterwarnings("ignore")

import operator

#Defining the possible steps
state1=(1, 0)
state2=(0, 1)
state3=(-1, 0)
state4=(0, -1)
state5=(1, 1)
state6=(1, -1)
state7=(-1, -1)
state8=(-1, 1)


ob=[]


#Function to create the next possible steps, for the robot
def valid_children(start,queue,ob=ob):
    ch=[]

    #All the possible actionable outcomes
    #1st number in the x co-ordinates of the Robot
    #2nd number in the y co-ordinates of the Robot
    

    for i in (state1,state2,state3,state4,state5,state6,state7,state8):
        branch = [a + b for a, b in zip(start, i)] #Adds the current state with the possible state
        branch = tuple(branch)
        temp=False
        for i in ob:
            a=find_obstacles((i[0],i[1]),(i[2],i[3]),(i[4],i[5]),(i[6],i[7]),(branch)) #Checking if obstacle is found or not. If found, skips and moves to the next possible step
            temp=temp or a 
        temp2=False
        for i in ob:
			edge = (edge_case((i[0],i[1]),(i[2],i[3]),(i[4],i[5]),(i[6],i[7]),(start)) and edge_case((i[0],i[1]),(i[2],i[3]),(i[4],i[5]),(i[6],i[7]),(branch)) and round(euc_d(start,branch),3)==1.414) #Checks for the edge case
			temp2=temp2 or edge

        if temp:
            continue
        if temp2:
        	continue

        elif any(positive(x) for x in branch): #Check for the upper bound 
            continue
        elif any(negative(x) for x in branch): #Check for the lower bound
            continue  
        elif (branch in queue): #Check for repeated states
            continue
        else:
            ch.append(branch) #Append only if it passes through all the constraints above

    return ch


def edge_case(a,b,c,d,p):
    if collinear(a,p,b) or collinear(b,p,c) or collinear(c,p,d) or collinear(d,p,a):
        return True


def find_obstacles(a,b,c,d,p): #a,b,c,d are vertices of the rectangle and p is the location of the AI robot
    a0=(np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)) * (np.sqrt((b[0]-c[0])**2 + (b[1]-c[1])**2)) #Area of the total rectangle=length * breadth
    semi1=0.5*(euc_d(a,p)+euc_d(p,b)+euc_d(a,b))
    semi2=0.5*(euc_d(b,p)+euc_d(p,c)+euc_d(b,c))
    semi3=0.5*(euc_d(c,p)+euc_d(p,d)+euc_d(d,c))
    semi4=0.5*(euc_d(a,p)+euc_d(p,d)+euc_d(a,d))
    #Checking if the point lies inside the rectangle by finding if it makes a valid traingle or not
    a1=np.sqrt((semi1*(semi1-euc_d(a,p))*(semi1-euc_d(p,b))*(semi1-euc_d(a,b))))
    a2=np.sqrt((semi2*(semi2-euc_d(b,p))*(semi2-euc_d(p,c))*(semi2-euc_d(b,c))))
    a3=np.sqrt((semi3*(semi3-euc_d(c,p))*(semi3-euc_d(p,d))*(semi3-euc_d(d,c))))
    a4=np.sqrt((semi4*(semi4-euc_d(a,p))*(semi4-euc_d(p,d))*(semi4-euc_d(a,d))))
	
	
	#To check for if it lies on edges of rectangle using collinearity

    if collinear(a,p,b) or collinear(b,p,c) or collinear(c,p,d) or collinear(d,p,a):
        return False
    
    if (round(a0,0)==round(a1+a2+a3+a4,0)):
        return True
    else:
        return False
    
def check_goal(start):
    return(start==goal)#Check for the goal state

def negative(a):
    return(a<0)#Check for the lower bound

def positive(a):
    return(a>50)#Check for the upper bound

def euc_d(a,b):
    return np.sqrt((a[1] - b[1])**2 + (a[0] - b[0])**2)#Check for the Euclidean distance
    
def collinear(a,p,b):
    return euc_d(a,b)==euc_d(a,p) + euc_d(p,b)#Check for collinearity

#Function to recursively select the best next step possible with the help of lowest total_cost
def blind_dfs(start,queue,path_cost,total_cost):

    queue.append(start)
    
    if check_goal(start):
    
        print ("Solution:")
        print('The cost is', total_cost)
        for q in queue:
            print (q)
        return True

    children=valid_children(start,queue,ob)
    cumulative={}
    for i in children:
        if ((tuple((a - b for a, b in zip(i, start)))==state1) or
            (tuple((a - b for a, b in zip(i, start)))==state2) or 
            (tuple((a - b for a, b in zip(i, start)))==state3) or
            (tuple((a - b for a, b in zip(i, start)))==state4)):
            path_cost=1.000 #Path cost for straight moves
        else:
            path_cost=1.414 #Path cost for diagonal moves
        cumulative.update({i : path_cost+euc_d(i,goal)}) #f=g+h
    
    
    queue.append((min(cumulative.items(), key=operator.itemgetter(1))[0]))
    path_cost=min(cumulative.values())
    
    
    
    total_cost=total_cost+path_cost
    blind_dfs(queue.pop(),queue,path_cost,total_cost)



def run():

    
    queue = [] #Queue to track the path
    path_cost=0 #Path cost which is g
	
	
    total_cost=0 #Total cost which is f
    #Initializer
    blind_dfs((0,0),queue,path_cost,total_cost)

	

#Driver code

datasets=["dataset1.txt","dataset2.txt","dataset3.txt"]

#Dataset 1 is easy dataset
#Dataset 2 is difficult dataset
#Dataset 3 is custom dataset with 6 obstacles




for idx,dataset in enumerate(datasets):
    lcount=0
    ob=[]
    print 'Dataset', idx+1
    f=open(dataset,'r')
    
    for x in f:
        if lcount==0:
            start=tuple(map(float, x.split(' '))) #Reads the start from file
        if lcount==1:
            goal=tuple(map(float, x.split(' '))) #Reads the goal from file
        if lcount>=3:
            ob.append(tuple(map(int, x.split(' ')))) #Appends all the coordinates of the obstacles to the list obs
            
        lcount+=1
    run()


