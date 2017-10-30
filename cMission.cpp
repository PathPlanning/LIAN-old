#include"cMission.h"
#include <ios>
#include <iomanip>

cMission::cMission(const char *fName) {
    m_fileName = fName;
    m_pSearch = 0;
    m_pLogger = 0;
}

cMission::~cMission() {
    delete m_pSearch;
    delete m_pLogger;
}

bool cMission::getMap() {
    return m_map.getMap(m_fileName);
}

bool cMission::getConfig() {
    return m_config.getConfig(m_fileName);
}

void cMission::createSearch() {
    m_pSearch = new LianSearch(
            (float) m_config.searchParams[CN_PT_AL],
            (int) m_config.searchParams[CN_PT_D],
            m_config.searchParams[CN_PT_W],
            (unsigned int) m_config.searchParams[CN_PT_SL],
            m_config.searchParams[CN_PT_CRF],
            m_config.searchParams[CN_PT_CHW],
            m_config.searchParams[CN_PT_DDF],
            (int) m_config.searchParams[CN_PT_DM],
            m_config.searchParams[CN_PT_LC],
            m_config.searchParams[CN_PT_CLC],
            (int) m_config.searchParams[CN_PT_NOP],
            (int) m_config.searchParams[CN_PT_BT]
    );
}

bool cMission::createLog() {
    if (m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_LOW || m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_HIGH
        || m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_MED) {
        m_pLogger = new cXmlLogger(m_config.searchParams[CN_PT_LOGLVL]);
    } else if (m_config.searchParams[CN_PT_LOGLVL] == CN_LOGLVL_NO) {
        m_pLogger = new cXmlLogger(m_config.searchParams[CN_PT_LOGLVL]);

        return true;
    } else {
        std::cout << "'loglevel' is not correctly specified in input XML-file.\n";
        return false;
    }

    return m_pLogger->getLog(m_fileName);
}

void cMission::startSearch() {
    sr = m_pSearch->startSearch(m_pLogger, m_map);
}

void cMission::printSearchResultsToConsole() {
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    if (sr.pathfound)
        std::cout << "pathlength=" << sr.pathlength << std::endl;
    int precision = std::cout.precision();
    std::cout << "time=" << std::setprecision(6) << std::fixed << sr.time << std::endl << std::setprecision(precision)
              << std::defaultfloat;
}

void cMission::saveSearchResultsToLog() {
    m_pLogger->writeToLogSummary(sr.hppath, sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, sr.maxAngle,
                                 sr.sections);

    if (sr.pathfound) {
        m_pLogger->writeToLogPath(sr.lppath, sr.angles);
        m_pLogger->writeToLogMap(m_map, sr.lppath);
        m_pLogger->writeToLogHpLevel(sr.hppath);
    }
    m_pLogger->saveLog();
}

