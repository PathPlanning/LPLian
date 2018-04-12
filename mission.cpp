#include "mission.h"
#include "xmllogger.h"
#include "gl_const.h"

Mission::Mission()
{
    logger = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    logger = nullptr;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
}

bool Mission::getMap()
{
    return map.GetMap(fileName);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != NULL) delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createSearch()
{
    dliansearch = DLian((double)config.SearchParams[CN_SP_AL], (int)config.SearchParams[CN_SP_DI], (float)config.SearchParams[CN_SP_HW],
                        (bool)config.SearchParams[CN_SP_PS]);
}

void Mission::startSearch()
{
    sr = dliansearch.FindThePath(map);
    if (sr.pathfound) {
        std::cout << "DLIAN has found the path\n";
    }
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "DLian path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;

    std::cout << "numberofsteps: " << sr.numberofsteps << std::endl;
    std::cout << "nodescreated: " << sr.nodescreated << std::endl;

    if (sr.pathfound) {
        std::cout << "pathlength: " << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled: " <<  sr.pathlength * map.get_cellsize() << std::endl;
    }
    std::cout << "time: " <<  sr.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.get_cellsize());
    if (sr.pathfound) {
        logger->writeToLogPath(sr.lppath);
        logger->writeToLogHPpath(sr.hppath);
        logger->writeToLogMap(map, sr.lppath, true);
    } else {
        logger->writeToLogMap(map, sr.lppath, false);
        logger->writeToLogNotFound();
    }
    logger->saveLog();
}
