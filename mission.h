#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "config.h"
#include "logger.h"
#include "searchresult.h"
#include "lplian.h"
#include "xmllogger.h"

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap();
        bool getConfig();
        bool createLog();
        void createSearch();
        void startSearch();
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();

    private:
        const char* getAlgorithmName();

        Map                     map;
        Config                  config;
        LPLian                   dliansearch;
        LPLian                   liansearch;
        ILogger*                logger;
        const char*             fileName;
        SearchResult            sr;
};

#endif

