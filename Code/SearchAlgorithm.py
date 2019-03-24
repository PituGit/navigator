# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = 'OscarLostes, EstevePineda, MaximoMartinez'
__group__ = 'DL15.08'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
import math


class Node:
    # __init__ Constructor of Node Class.
    def __init__(self, station, father):
        """
        __init__: 	Constructor of the Node class
        :param
                - station: STATION information of the Station of this Node
                - father: NODE (see Node definition) of his father
        """

        self.station = station  # STATION information of the Station of this Node
        self.father = father  # NODE pointer to his father

        if father == None:
            self.parentsID = []
        else:
            self.parentsID = [father.station.id]
            self.parentsID.extend(father.parentsID)  # TUPLE OF NODES (from the origin to its father)

        self.g = 0  # REAL cost - depending on the type of preference -
        # to get from the origin to this Node
        self.h = 0  # REAL heuristic value to get from the origin to this Node
        self.f = 0  # REAL evaluate function
        self.time = 0  # REAL time required to get from the origin to this Node
        # [optional] Only useful for GUI
        self.num_stopStation = 0  # INTEGER number of stops stations made from the origin to this Node
        # [optional] Only useful for GUI
        self.distance = 0  # REAL distance made from the origin to this Node
        # [optional] Only useful for GUI
        self.transfers = 0  # INTEGER number of transfers made from the origin to this Node
        # [optional] Only useful for GUI

    def setEvaluation(self):
        """
        setEvaluation: 	Calculates the Evaluation Function. Actualizes .f value

        """
        self.f = self.g + self.h

    def setHeuristic(self, typePreference, station_destination, city):
        """"
        setHeuristic: 	Calculates the heuristic depending on the preference selected
        :params
                - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
                - station_destination: PATH of the destination station
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        """
        
        x_coord = ((self.station.x)-(station_destination.station.x))**2
        y_coord = ((self.station.y)-(station_destination.station.y))**2
        tmp = 0
        var = False
        distance = math.sqrt(x_coord+y_coord)

        if(typePreference == 1):
            if (self.station.line != station_destination.station.line):
                for j in city.StationList:
                    for i in city.StationList:
                        if(i.name == j.name and i.id != j.id and i.line != j.line and i.line == station_destination.station.line and var !=True):
                            var = True
                            tmp = city.StationList[j.id-1].destinationDic[i.id]
                self.h = (distance / city.max_velocity) + tmp
            else:
                self.h = distance / city.max_velocity
        elif(typePreference == 2):
            self.h = distance
        elif(typePreference == 3):
            if(self.station.line == station_destination.station.line):
                self.h = 0
            else:
                self.h = 1
        elif(typePreference == 4):
            if(len(self.station.destinationDic) != 0):
                self.h = 1

    def setRealCost(self, costTable):
        """
        setRealCost: 	Calculates the real cost depending on the preference selected
        :params
                 - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
        """

        self.g = 0
        
        origin = self.father
        destination = self
        while origin:
            self.g += costTable[origin.station.id][destination.station.id]

            destination = origin
            origin = origin.father


def coord2station(coord, stationList):
    """
    coord2station :      From coordinates, it searches the closest station.
    :param
            - coord:  LIST of two REAL values, which refer to the coordinates of a point in the city.
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)

    :return:
            - possible_origins: List of the Indexes of the stationList structure, which corresponds to the closest
            station
    """

    distance = []
    for station in stationList:
        distance.append((station.id-1, math.sqrt(math.pow(station.x - coord[0],2) + math.pow(station.y - coord[1],2))))
    
    distance.sort(key = lambda x: x[1])

    possible_origins = []
    for station in distance:
        if station[1] == distance[0][1]:
            possible_origins.append(station[0] - 1)
  
    return possible_origins


def Expand(fatherNode, city, station_destination=None, typePreference=0, costTable=None):
    """
        Expand: It expands a node and returns the list of connected stations (childrenList)
        :params
                - fatherNode: NODE of the current node that should be expanded
                - city: CITYINFO with the information of the city (see CityInfo class definition)
                - node_destination: NODE (see Node definition) of the destination
                - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
                - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.

        :returns
                - childrenList:  LIST of the set of child Nodes for this current node (fatherNode)

    """
    
    childrenList = []
    for i in fatherNode.station.destinationDic.keys():
        child = Node(city.StationList[i-1], fatherNode)

        if costTable:
            child.setRealCost(costTable)
            child.setHeuristic(typePreference, station_destination, city)
            child.setEvaluation()

            if typePreference == 1:
                child.time = child.g
            elif typePreference == 2:
                child.distance = child.g
            elif typePreference == 3:
                child.transfers = child.g
            elif typePreference == 4:
                child.num_stopStation = child.g

        childrenList.append(child)    

    return childrenList
        


def RemoveCycles(childrenList):
    """
        RemoveCycles: It removes from childrenList the set of childrens that include some cycles in their path.
        :params
                - childrenList: LIST of the set of child Nodes for a certain Node
        :returns
                - listWithoutCycles:  LIST of the set of child Nodes for a certain Node which not includes cycles
    """
    
    listWithoutCycles = []
    for children in childrenList:
        if children.station.id not in children.parentsID:
            listWithoutCycles.append(children)

    return listWithoutCycles



def RemoveRedundantPaths(childrenList, nodeList, partialCostTable):
    """
        RemoveRedundantPaths:   It removes the Redundant Paths. They are not optimal solution!
                                If a node is visited and have a lower g in this moment, TCP is updated.
                                In case of having a higher value, we should remove this child.
                                If a node is not yet visited, we should include to the TCP.
        :params
                - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
                - nodeList : LIST of NODES to be visited
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node
        :returns
                - childrenList: LIST of NODES, set of childs without rendundant path.
                - nodeList: LIST of NODES to be visited updated (without redundant paths)
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node (updated)
    """

    for child in childrenList:
        if child.station.id in partialCostTable.keys():
            if partialCostTable[child.station.id] > child.g:
                partialCostTable[child.station.id] = child.g
                for node in nodeList:
                    if child.station.id == node.station.id:
                        nodeList.remove(node)
            else:
                childrenList.remove(child)
        else:
            partialCostTable[child.station.id] = child.g
    return childrenList, nodeList, partialCostTable


def sorted_insertion(nodeList, childrenList):
    """ Sorted_insertion: 	It inserts each of the elements of childrenList into the nodeList.
                            The insertion must be sorted depending on the evaluation function value.

        : params:
            - nodeList : LIST of NODES to be visited
            - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
        :returns
                - nodeList: sorted LIST of NODES to be visited updated with the childrenList included
    """
    partialCostTable = {}
    childrenList, nodeList, partialCostTable = RemoveRedundantPaths(childrenList, nodeList, partialCostTable)
    newNodeList = list(nodeList)
    for i in childrenList:
        newNodeList.append(i)
    newNodeList.sort(key = lambda x: x.f)
    return newNodeList


def setCostTable(typePreference, city):
    """
    setCostTable :      Real cost of a travel.
    :param
            - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - city: CITYINFO with the information of the city (see CityInfo class definition)
    :return:
            - costTable: DICTIONARY. Relates each station with their adjacency an their g, depending on the
                                 type of Preference Selected.
    """
   
    costTable = {}
    for station1 in city.StationList:
        costTable[station1.id] = {}
        for station2 in station1.destinationDic.keys():
            if typePreference == 0:
                costTable[station1.id][station2] = 1
            elif typePreference == 1:
                costTable[station1.id][station2] = station1.destinationDic[station2]
            elif typePreference == 2:
                if station1.line == city.StationList[station2 - 1].line:
                    costTable[station1.id][station2] = station1.destinationDic[station2 ] * city.velocity_lines[station1.line - 1]
                else:
                    costTable[station1.id][station2] = 0
            elif typePreference == 3:
                if station1.line != city.StationList[station2 - 1].line:
                    costTable[station1.id][station2] = 1
                else:
                    costTable[station1.id][station2] = 0
            elif typePreference == 4:
                if station1.name != city.StationList[station2 - 1].name:
                    costTable[station1.id][station2] = 1
                else:
                    costTable[station1.id][station2] = 0

    
    return costTable


def AstarAlgorithm(coord_origin, coord_destination, typePreference, city, flag_redundants):
    """
     AstarAlgorithm: main function. It is the connection between the GUI and the AStar search code.
     INPUTS:
            - stationList: LIST of the stations of a city. (- id, name, destinationDic, line, x, y -)
            - coord_origin: TUPLE of two values referring to the origin coordinates
            - coord_destination: TUPLE of two values referring to the destination coordinates
            - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - city: CITYINFO with the information of the city (see CityInfo class definition)
			- flag_redundants: [0/1]. Flag to indicate if the algorithm has to remove the redundant paths (1) or not (0)

    OUTPUTS:
            - time: REAL total required time to make the route
            - distance: REAL total distance made in the route
            - transfers: INTEGER total transfers made in the route
            - stopStations: INTEGER total stops made in the route
            - num_expanded_nodes: INTEGER total expanded nodes to get the optimal path
            - depth: INTEGER depth of the solution
            - visitedNodes: LIST of INTEGERS, IDs of the stations corresponding to the visited nodes
            - idsOptimalPath: LIST of INTEGERS, IDs of the stations corresponding to the optimal path
            (from origin to destination)
            - min_distance_origin: REAL the distance of the origin_coordinates to the closest station
            - min_distance_destination: REAL the distance of the destination_coordinates to the closest station



            EXAMPLE:
            return optimalPath.time, optimalPath.walk, optimalPath.transfers,optimalPath.num_stopStation,
            len(expandedList), len(idsOptimalPath), visitedNodes, idsOptimalPath, min_distance_origin,
            min_distance_destination
    """

    """ typePreference = int(typePreference)
    
    costTable = setCostTable(typePreference, city)

    idArrel = coord2station(coord_origin, city.StationList)[0]
    idObjectiu = coord2station(coord_destination, city.StationList)[0]
    
    nodeArrel = Node(city.StationList[idArrel + 1], None)
    nodeObjectiu = Node(city.StationList[idObjectiu + 1], None)

    nodeArrel.setRealCost(costTable)
    nodeArrel.setHeuristic(typePreference, nodeObjectiu, city)
    nodeArrel.setEvaluation()

    nodeObjectiu.setRealCost(costTable)
    nodeObjectiu.setHeuristic(typePreference, nodeObjectiu, city)
    nodeObjectiu.setEvaluation()

    #A*
    llista = [[nodeArrel]]

    while (llista[0][0].station.id != nodeObjectiu.station.id) or (llista == []):
        c = llista[0]
        e = Expand(c[0], city, nodeObjectiu, typePreference, costTable)
        e = RemoveCycles(e)
        llista.append(sorted_insertion(llista, e))

    if llista:
        #return time, distance, transfers,
        #   stopStations, expanded_nodes, num_depth,
        #   visited_nodes, idoptimalpath, min_distance_origin,
        #   min_distance_destination
        return 0.0, 0.0, 0, 0, 0, 0, [],  [1, 3, 5], 0.0, 0.0
    else:
        return "No existeix solució" """

    typePreference = int(typePreference)
    partialCostTable = {} 
    CostTable = setCostTable(typePreference, city)

    NodeOrigen = Node(city.StationList[coord2station(coord_origin, city.StationList)[0] + 1], None)
    NodeDesti = Node(city.StationList[coord2station(coord_destination, city.StationList)[0] + 1], None)
    
    NodeOrigen.setHeuristic(typePreference, NodeDesti, city) 
    NodeOrigen.setRealCost(CostTable) 
    NodeOrigen.setEvaluation()
    
    llista = [NodeOrigen]
    
    Nodo=llista[0]
    nodosvi=[]
    camioptim=[]
    while ((Nodo.station.id != NodeDesti.station.id) or (llista == None)):         
        c=llista.pop(0)
        nodosvi.append(c.station.id)
        
        e = Expand(c, city, NodeDesti, typePreference, CostTable) 
        e = RemoveCycles(e)
        llista = sorted_insertion(llista, e)
        
        Nodo = llista[0]

    nodosvi.append(Nodo.station.id)
    camioptim = Nodo.parentsID[::-1]
    camioptim.append(Nodo.station.id)
    minorigen = math.sqrt ((coord_origin[0]-NodeOrigen.station.x) ** 2 + (coord_origin[1]-NodeOrigen.station.y) ** 2 )
    mindesti = math.sqrt ((coord_destination[0]-NodeDesti.station.x) ** 2 + (coord_destination[1]-NodeDesti.station.y) ** 2 )
   
    return Nodo.time, Nodo.distance, Nodo.transfers, Nodo.num_stopStation, len(nodosvi), len(camioptim), nodosvi, camioptim, minorigen, mindesti

