# With this exercise we will work on finding a station of the path given certain coordinates
#
__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2018- 2019
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

    coord=[67,79]
    coord1=[140,20]
    coord2=[100,199]
    #------------------------------------------------------------------#
    # Code the function "coord2station" in SearchAlgorithm.py

    node=coord2station(coord,city.StationList)
    node1=coord2station(coord1,city.StationList)
    node2=coord2station(coord2,city.StationList)

    print("\nFrom coord: " + str(coord) + " possible nodes: ")
    for i in node:
        print(stationList[i].name + " L"+ str(stationList[i].line) + " with ID: " + str([stationList[i].id]))




    print("\nFrom coord: " + str(coord1) + " possible nodes:  ")
    for i in node1:
        print(stationList[i].name + " L"+ str(stationList[i].line) + " with ID: " + str([stationList[i].id]))


    print("\nFrom coord: " + str(coord2) + " possible nodes: ")
    for i in node2:
        print(stationList[i].name + " L"+ str(stationList[i].line) + " with ID: " + str([stationList[i].id]))



if __name__ == '__main__':
    main()