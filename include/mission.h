#ifndef MISSION_H
#define MISSION_H

#include "map.h"
#include "config.h"
#include "xmlLogger.h"
#include "searchresult.h"
#include "aa_sipp.h"
#include "task.h"
#include "dynamicobstacles.h"

namespace SIPP {
    class Mission {
    public:
        Mission();

        ~Mission();

        bool getMap();

        void setMap(Map map);

        bool getTask();

        void setTask(Task task);

        bool getConfig();

        void setConfig(Config config);

        bool getObstacles();

        void setObstacles(DynamicObstacles dynamicObstacles);

        void createLog();

        void createSearch();

        void startSearch();

        void printSearchResultsToConsole();

        void saveSearchResultsToLog();

        SearchResult getSearchResult();

        void setFileNames(const char *taskName, const char *mapName, const char *configName, const char *obstaclesName);

    private:
        Map m_map;
        Task m_task;
        Config m_config;
        DynamicObstacles m_obstacles;
        AA_SIPP *m_pSearch;
        XmlLogger *m_pLogger;
        SearchResult sr;
        const char *mapName;
        const char *taskName;
        const char *configName;
        const char *obstaclesName;

    };
}

#endif

