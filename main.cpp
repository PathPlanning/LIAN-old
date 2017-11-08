#include"mission.h"

#include <iostream>
int main(int argc, char* argv[])
{
    if (argc==2)
        {
            Mission mission(argv[1]);


            std::cout<<"Retreiving map from input XML file.\n";
            if (!mission.getMap())
            {
                std::cout<<"Program terminated.\n";
                return 0;
            }

            std::cout<<"Retreiving search algorithm configuration from input XML file.\n";
            if (!mission.getConfig())
            {
                return 0;
            }

            mission.createSearch();
            mission.createLog();
            mission.startSearch();

            std::cout<<"Search is finished!"<<std::endl;

            mission.printSearchResultsToConsole();

            mission.saveSearchResultsToLog();
            std::cout<<"Results are saved (if chosen) via created log channel."<<std::endl;
        }

    return 1;
}
