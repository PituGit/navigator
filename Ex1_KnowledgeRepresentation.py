# With this exercise we will work on understanding the representation of the knowledge.

__authors__='TO_BE_FILLED'
__group__='DL01'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2018- 2019
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________


import os
from SubwayMap import *

def main():
    city_string="Lyon_smallCity"
    # read file of the Station information
    print("CITY INFORMATION   *******************************************")
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation",city_string,"Stations.txt")
    stationList = readStationInformation(filename)
    # Print information of the stations
    print_stationList(stationList)


    #Station Time Cost Table
    print("STATION TIME COST TABLE *******************************************")
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation",city_string,"Time.txt")
    timeStations = readCostTable(filename)
    print(" Real time between two different stations:")
    print_connections(timeStations)

    # Velocity of each line
    filename = os.path.join(os.path.dirname(__file__),"..","CityInformation",city_string,"InfoVelocity.txt")
    infoVelocity = readInformation(filename)
    print(" Metro Lines run at velocity: ")
    print(infoVelocity)
    print(" max. velocity = " + str(max(infoVelocity)))
    print(" min. velocity = " + str(min(infoVelocity)))
    print("\n")


    #we save the global information of the city in 'city' variable.
    city=CityInfo(infoVelocity,stationList,timeStations)
    # Now we have to set the values of the dictionary for each station (destinationDic) with the values in timeStations
    print_dictionary(city.StationList)

    # _________________________________________________________________________________________

     # Take a look and understand how the classes and functions included in SubwayMap.py
     # Use this space below to write your code to answer Ex 1.
    

    transfers = search_multiple_lines(city.StationList)
    print_dictionary(transfers)







    # _______________________________________________________________________________________



if __name__ == '__main__':
    main()


