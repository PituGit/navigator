# With this exercise we will work on the evaluation function of the algorithm.
#
__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________
from SearchAlgorithm import *
from SubwayMap import *
import os


def main():

    #------------------------------------------------------------------#
    city_string="Lyon_smallCity"

    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "Stations.txt")
    stationList = readStationInformation(filename)


    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "Time.txt")
    timeStations = readCostTable(filename)


    # Now we want to analyze extra information from the files.
    # Velocity of the lines
    filename = os.path.join(os.path.dirname(__file__), "..", "CityInformation", city_string, "InfoVelocity.txt")
    infoVelocity = readInformation(filename)


    # we save the global information of the city in 'city' variable.
    # Analyze which information contains an element of the class CityInfo.
    city = CityInfo(infoVelocity, stationList, timeStations)

    #------------------------------------------------------------------#

    origin=Node(stationList[1],None)            # Charpennes L1
    child=Node(stationList[4], origin)          # Charpennes L2
    destination=Node(stationList[13],None)      # Dauphine lacassagne L4

    #------------------------------------------------------------------#


    #HEURISTIC 1 : TIME
    timeCostTable=setCostTable(1, city)
    child.setHeuristic(1,destination,city)
    child.setRealCost( timeCostTable)
    child.setEvaluation()
    print(" Evaluation Function (TIME) from " + child.station.name + " L" + str(child.station.line) + " to " + destination.station.name + " L" + str(destination.station.line) +  ": \t\t" +  str(child.f))
    print("                               g : " + str(child.g))
    print("                               h : " + str(child.h) + "\n")

    #HEURISTIC 2 : DISTANCE
    distCostTable=setCostTable( 2,  city)
    child.setHeuristic(2,destination,city)
    child.setRealCost( distCostTable)
    child.setEvaluation()
    print(" Evaluation Function (DISTANCE) from " + child.station.name + " L" + str(child.station.line) + " to " + destination.station.name + " L" + str(destination.station.line) +  ": \t\t" +  str(child.f))
    print("                               g : "  + str(child.g))
    print("                               h : " + str(child.h) + "\n")

    #HEURISTIC 3 : TRANSFERS
    transCostTable=setCostTable( 3, city)
    child.setHeuristic(3,destination,city)
    child.setRealCost( transCostTable)
    child.setEvaluation()
    print(" Evaluation Function (#TRANSFERS) from " + child.station.name + " L" + str(child.station.line) + " to " + destination.station.name + " L" + str(destination.station.line) +  ": \t\t" +  str(child.f))
    print("                               g : " + str(child.g))
    print("                               h : " + str(child.h) + "\n")

    #HEURISTIC 4 : STOP STATIONS
    stopCostTable=setCostTable(4, city)
    child.setHeuristic(4,destination,city)
    child.setRealCost(  stopCostTable)
    child.setEvaluation()
    print(" Evaluation Function (#STATIONS) from " + child.station.name + " L" + str(child.station.line) + " to " + destination.station.name + " L" + str(destination.station.line) +  ": \t\t" +  str(child.f))
    print("                               g : " + str(child.g))
    print("                               h : " + str(child.h) + "\n")



if __name__ == '__main__':
    main()