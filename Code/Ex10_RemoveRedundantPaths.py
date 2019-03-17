# TO TEST setHeuristic FUNCTION
#
__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

import os
import sys
import heapq

from SearchAlgorithm import *

from SubwayMap import *

def main():
    # ------------------------------------------------------------------#
    city_string = "Lyon_smallCity"
    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "Stations.txt")
    stationList = readStationInformation(filename)

    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "Time.txt")
    timeStations = readCostTable(filename)

    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "InfoVelocity.txt")
    infoVelocity = readInformation(filename)

    city = CityInfo(infoVelocity, stationList, timeStations)

    # ------------------------------------------------------------------#
    typePreference=int(1)
    nodeList=[]
    partialCostTable = {}
    currentCostTable=setCostTable( typePreference,city)
	
	
	
	#Imagine that we have not explored anything. This is the first time that we analyze a specific node.
	#     - It means that the TCP is empty.
	#	  - For example, we suppose that we are starting our search algorithm on Charpennes L1 as the origin
	#	  - We set g and h with random values
	
	#nodeList is a list of Nodes to be visited (set of childs of several nodes that never have been expanded.)
    origin = Node(stationList[1], None)  # Charpennes L1
    destination = Node(stationList[13], None)  # Dauphine lacassagne L4
    origin.g=0
    origin.h=500
    nodeList=[]
    nodeList.append(origin)
    print("\n ---------------------  (1)  ----------------- \n"	)
    print("\n NODELIST :")
	
    for i in nodeList:
        print("        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
	# Now we have to analyze the head of our list and remove it from the list:
    current = nodeList[0]
    nodeList = nodeList[1:]

	# Expand head of the list
    childrenList=Expand(current, city,destination,typePreference=typePreference,  costTable=currentCostTable)

	# Look to its childrens:
	#		- It is recommended to set .g and .h values inside Expand function for each child. 
	#		- You should have values inside i.g in the following print
    print("\n CHILDRENLIST   ")
    for i in childrenList:
        print("        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))

	#Remove Redundant paths
    childrenList, nodeList, partialCostTable = RemoveRedundantPaths(childrenList, nodeList, partialCostTable)
	
	# Look to the TCP:
    print( "\n TCP  (after removing Redundant Paths): ")
    for i in partialCostTable.keys():
        print( "        ID : " + str(i) + "    cost: " + str(partialCostTable[i]))
	
    print( "\n NODELIST  (after removing Redundant Paths): ")
	
    for i in nodeList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
    print( "\n CHILDRENLIST (after removing Redundant Paths):  ")
    for i in childrenList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	

	#add childs into the list for visiting. IN THIS EXAMPLE, WE DO NOT INSTERT THEM SORTEDLY!
    for child in childrenList:
        nodeList.append(child)
    print("\n NODELIST  (after adding childs): ")
	
    for i in nodeList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
    print( "\n ---------------------  (2)  ----------------- \n"	)
	# Now we have to analyze the next head of our list:
    current = nodeList[0]
    nodeList = nodeList[1:]

    # Expand head of the list
    childrenList = Expand(current, city, destination, typePreference=typePreference, costTable=currentCostTable)

	# Look to its childrens:
    print( "\n CHILDRENLIST  of the node :  "  + str(current.station.id))
    for i in childrenList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
	#Remove Redundant paths
    childrenList, nodeList, partialCostTable = RemoveRedundantPaths(childrenList, nodeList, partialCostTable)
	
	# Look to the TCP:
	#    - Compare current TCP with the previous one. You should have another entry and previous entries, should have the same associated cost.
    print( "\n TCP  (after removing Redundant Paths): ")
    for i in partialCostTable.keys():
        print( "        ID : " + str(i) + "    cost: " + str(partialCostTable[i]))
	
	#Look to Nodelist
	#	- compare with the previous one
	#	- one item should be removed (why?)
    print( "\n NODELIST  (after removing Redundant Paths): ")
    for i in nodeList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
	#Look to ChildrenList
	#	- compare with the previous one
	#	- We should have exactly the same childrenList as before removing reduntant paths
    print( "\n CHILDRENLIST (after removing Redundant Paths):  ")
    for i in childrenList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
	#add childs into the list for visiting. IN THIS EXAMPLE, WE DO NOT INSTERT THEM SORTEDLY!
    for child in childrenList:
        nodeList.append(child)
    
	
	#Look to Nodelist
	#	- compare with the previous one
	#	- one item should be added
    print( "\n NODELIST  (after adding childs): ")
	
    for i in nodeList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
		
    print( "\n ---------------------  (3)  ----------------- \n"	)
	# Now we have to analyze the head of our list:
    current = nodeList[0]
    nodeList = nodeList[1:]

    # Expand head of the list
    childrenList = Expand(current, city, destination, typePreference=typePreference, costTable=currentCostTable)

	# Look to its childrens:
    print( "\n CHILDRENLIST  of the node :  "  + str(current.station.id))
    for i in childrenList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
	#Remove Redundant paths

    childrenList, nodeList, partialCostTable = RemoveRedundantPaths(childrenList, nodeList, partialCostTable)

	# Look to the TCP:
	#    - Compare current TCP with the previous one. You should have more entries. (why?)
    print( "\n TCP  (after removing Redundant Paths): ")
    for i in partialCostTable.keys():
        print( "        ID : " + str(i) + "    cost: " + str(partialCostTable[i]))
	
	#Look to Nodelist
	#	- compare with the previous one
	#	- one item should be removed (why?)
    print( "\n NODELIST  (after removing Redundant Paths): ")
    for i in nodeList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	
	#Look to ChildrenList
	#	- compare with the previous one
	#	- Two items should be removed. (why?)
    print( "\n CHILDRENLIST (after removing Redundant Paths):  ")
    for i in childrenList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))
	#add childs into the list for visiting. IN THIS EXAMPLE, WE DO NOT INSTERT THEM SORTEDLY!
    for child in childrenList:
        nodeList.append(child)
 
	#Look to Nodelist
	#	- compare with the previous one
	#	- one item should be added
    print( "\n NODELIST  (after adding childs): ")
	
    for i in nodeList:
        print( "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(i.g))

    print("\n ---------------------  (4)  ----------------- \n")
    # Now we have to analyze the head of our list:
    current = nodeList[0]
    nodeList = nodeList[1:]

    # Expand head of the list
    childrenList = Expand(current, city, destination, typePreference=typePreference, costTable=currentCostTable)

    # Look to its childrens:
    print("\n CHILDRENLIST  of the node :  " + str(current.station.id))
    for i in childrenList:
        print(
            "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(
                i.g))

    # Remove Redundant paths
    childrenList, nodeList, partialCostTable = RemoveRedundantPaths(childrenList, nodeList, partialCostTable)

    # Look to the TCP:
    #    - Compare current TCP with the previous one. You should have more entries. (why?)
    print("\n TCP  (after removing Redundant Paths): ")
    for i in partialCostTable.keys():
        print("        ID : " + str(i) + "    cost: " + str(partialCostTable[i]))

    # Look to Nodelist
    #	- compare with the previous one
    #	- one item should be removed (why?)
    print("\n NODELIST  (after removing Redundant Paths): ")
    for i in nodeList:
        print(
            "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(
                i.g))

    # Look to ChildrenList
    #	- compare with the previous one
    #	- Two items should be removed. (why?)
    print("\n CHILDRENLIST (after removing Redundant Paths):  ")
    for i in childrenList:
        print(
            "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(
                i.g))
    # add childs into the list for visiting. IN THIS EXAMPLE, WE DO NOT INSTERT THEM SORTEDLY!
    for child in childrenList:
        nodeList.append(child)

    # Look to Nodelist
    #	- compare with the previous one
    #	- one item should be added
    print("\n NODELIST  (after adding childs): ")

    for i in nodeList:
        print(
            "        " + str(i.station.id) + "     --> parents.ID : " + str(i.parentsID) + "   asoc. cost: " + str(
                i.g))


		
if __name__ == '__main__':
    main()