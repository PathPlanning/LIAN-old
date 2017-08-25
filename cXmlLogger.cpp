#include"cXmlLogger.h"
#include"gl_const.h"

#include <list>
#include <sstream>
#include <string>

cXmlLogger::cXmlLogger(float loglvl) {
    loglevel = loglvl;
    LogFileName = "";
    doc = 0;
}

cXmlLogger::~cXmlLogger() {
    if (doc) {
        doc->Clear();
        delete doc;
    }
}

bool cXmlLogger::getLog(const char *FileName) {
    std::string value;
    TiXmlDocument doc_xml(FileName);

    if (!doc_xml.LoadFile()) {
        std::cout << "Error opening XML-file in getLog";
        return false;
    }

    value = FileName;
    size_t dotPos = value.find_last_of(".");

    if (dotPos != std::string::npos) {
        value.insert(dotPos, CN_LOG);
    } else {
        value += CN_LOG;
    }

    LogFileName = value;
    doc_xml.SaveFile(LogFileName.c_str());

    doc = new TiXmlDocument(LogFileName.c_str());
    doc->LoadFile();

    TiXmlElement *msg;
    TiXmlElement *root;

    root = doc->FirstChildElement(CNS_TAG_ROOT);
    TiXmlElement *log = new TiXmlElement(CNS_TAG_LOG);
    root->LinkEndChild(log);

    msg = new TiXmlElement(CNS_TAG_MAPFN);
    msg->LinkEndChild(new TiXmlText(FileName));
    log->LinkEndChild(msg);

    msg = new TiXmlElement(CNS_TAG_SUM);
    log->LinkEndChild(msg);

    TiXmlElement *path = new TiXmlElement(CNS_TAG_PATH);
    log->LinkEndChild(path);

    TiXmlElement *angles = new TiXmlElement(CNS_TAG_ANGLES);
    log->LinkEndChild(angles);

    TiXmlElement *lplevel = new TiXmlElement(CNS_TAG_LPLEVEL);
    log->LinkEndChild(lplevel);

    TiXmlElement *hplevel = new TiXmlElement(CNS_TAG_HPLEVEL);
    log->LinkEndChild(hplevel);

    if (loglevel >= CN_LOGLVL_MED) {
        TiXmlElement *lowlevel = new TiXmlElement(CNS_TAG_LOWLEVEL);
        log->LinkEndChild(lowlevel);
    }

    return true;
}

void cXmlLogger::writeToLogMap(const cMap &Map, const std::list<Node> &path) {
    if (loglevel == CN_LOGLVL_NO) return;
    std::stringstream stream;
    std::string text, value;
    int *curLine;

    curLine = new int[Map.width];

    for (int i = 0; i < Map.width; i++)
        curLine[i] = 0;

    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_PATH);
    TiXmlElement *msg;

    for (int i = 0; i < Map.height; i++) {
        msg = new TiXmlElement(CNS_TAG_ROW);
        msg->SetAttribute(CNS_TAG_ATTR_NUM, i);
        text = "";

        for (auto iter = path.begin(); iter != path.end(); ++iter) {
            if (iter->i == i) {
                curLine[iter->j] = 1;
            }
        }

        for (int j = 0; j < Map.width; j++) {
            if (curLine[j] != 1) {
                stream << Map.Grid[i][j];
                stream >> value;
                stream.clear();
                stream.str("");
                text = text + value + " ";
            } else {
                text = text + "*" + " ";
                curLine[j] = 0;
            }
        }

        msg->LinkEndChild(new TiXmlText(text.c_str()));
        element->LinkEndChild(msg);
    }

    delete[] curLine;
}

void
cXmlLogger::writeToLogOpenClose(const iOpen *open,
                                const std::unordered_multiset<Node, std::hash<Node>, NodeCoordEqual> &close,
                                const int size) {

    if (loglevel == CN_LOGLVL_NO || loglevel == CN_LOGLVL_HIGH) return;

    int iterate = 0;
    TiXmlElement *element = new TiXmlElement(CNS_TAG_STEP);
    TiXmlNode *child = 0, *lowlevel = doc->FirstChild(CNS_TAG_ROOT);
    lowlevel = lowlevel->FirstChild(CNS_TAG_LOG)->FirstChild(CNS_TAG_LOWLEVEL);

    while (child = lowlevel->IterateChildren(child))
        iterate++;

    element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);
    lowlevel->InsertEndChild(*element);
    lowlevel = lowlevel->LastChild();

    element = new TiXmlElement(CNS_TAG_OPEN);
    lowlevel->InsertEndChild(*element);
    child = lowlevel->LastChild();

    Node min = open->FindMin();
    if (min.F != -1) {
        element = new TiXmlElement(CNS_TAG_NODE);
        element->SetAttribute(CNS_TAG_ATTR_X, min.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element->SetAttribute(CNS_TAG_ATTR_Z, min.z);
        element->SetDoubleAttribute(CNS_TAG_ATTR_F, min.F);
        element->SetDoubleAttribute(CNS_TAG_ATTR_G, min.g);
        element->SetAttribute(CNS_TAG_ATTR_PARX, min.Parent->j);
        element->SetAttribute(CNS_TAG_ATTR_PARY, min.Parent->i);
        element->SetAttribute(CNS_TAG_ATTR_PARZ, min.Parent->z);
        child->InsertEndChild(*element);
    }
    
    for (Node node : open->dump()) {
        element->Clear();
        element->SetAttribute(CNS_TAG_ATTR_X, node.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, node.i);
        element->SetAttribute(CNS_TAG_ATTR_Z, node.z);
        element->SetDoubleAttribute(CNS_TAG_ATTR_F, node.F);
        element->SetDoubleAttribute(CNS_TAG_ATTR_G, node.g);
        if (node.g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, node.Parent->j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, node.Parent->i);
            element->SetAttribute(CNS_TAG_ATTR_PARZ, node.Parent->z);
        }
        child->InsertEndChild(*element);
    }

    element = new TiXmlElement(CNS_TAG_CLOSE);
    lowlevel->InsertEndChild(*element);
    child = lowlevel->LastChild();

    for (auto it = close.begin(); it != close.end(); ++it) {
        element = new TiXmlElement(CNS_TAG_NODE);
        element->SetAttribute(CNS_TAG_ATTR_X, it->j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->i);
        element->SetAttribute(CNS_TAG_ATTR_Z, it->z);
        element->SetDoubleAttribute(CNS_TAG_ATTR_F, it->F);
        element->SetDoubleAttribute(CNS_TAG_ATTR_G, it->g);
        if (it->g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, it->Parent->j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, it->Parent->i);
            element->SetAttribute(CNS_TAG_ATTR_PARZ, it->Parent->z);
        }
        child->InsertEndChild(*element);
    }
}

void cXmlLogger::writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles) {
    if (loglevel == CN_LOGLVL_NO) return;
    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_LPLEVEL);

    TiXmlElement *point;

    int i = 0;

    for (auto iter = path.begin(); iter != path.end(); ++iter) {
        point = new TiXmlElement(CNS_TAG_NODE);
        point->SetAttribute(CNS_TAG_ATTR_NUM, i);
        point->SetAttribute(CNS_TAG_ATTR_X, iter->j);
        point->SetAttribute(CNS_TAG_ATTR_Y, iter->i);
        point->SetAttribute(CNS_TAG_ATTR_Z, iter->z);
        element->LinkEndChild(point);
        i++;
    }

    if (angles.size() == 0) {
        return;
    }

    element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_ANGLES);

    std::vector<float>::const_iterator iter2 = angles.end();
    for (i = 0; i < angles.size(); i++) {
        iter2--;
        point = new TiXmlElement(CNS_TAG_ANGLE);
        point->SetAttribute(CNS_TAG_ATTR_NUM, i);
        point->SetDoubleAttribute(CNS_TAG_ATTR_VALUE, *iter2);
        element->LinkEndChild(point);
    }
}

void cXmlLogger::saveLog() {
    if (loglevel == CN_LOGLVL_NO) return;
    doc->SaveFile(LogFileName.c_str());
}

void cXmlLogger::writeToLogHpLevel(const std::list<Node> &path) {
    if (loglevel == CN_LOGLVL_NO) return;
    int partnumber = 0;
    TiXmlElement *part;
    TiXmlElement *hplevel = doc->FirstChildElement(CNS_TAG_ROOT);
    hplevel = hplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_HPLEVEL);
    std::list<Node>::const_iterator iter = path.begin();
    std::list<Node>::const_iterator it = path.begin();

    while (iter != --path.end()) {
        part = new TiXmlElement(CNS_TAG_SECTION);
        part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
        part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
        part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
        part->SetAttribute(CNS_TAG_ATTR_SZ, it->z);
        iter++;
        part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
        part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
        part->SetAttribute(CNS_TAG_ATTR_FZ, iter->z);
        part->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
        hplevel->LinkEndChild(part);
        it++;
        partnumber++;
    }
}

void
cXmlLogger::writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length,
                              long double Time, float maxAngle, int sections) {
    if (loglevel == CN_LOGLVL_NO) return;
    std::string timeValue;
    std::stringstream stream;
    stream << Time;
    stream >> timeValue;
    stream.clear();
    stream.str("");
    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_SUM);

    if (path.empty()) {
        element->SetAttribute(CNS_TAG_ATTR_PF, CNS_TAG_ATTR_FALSE);
    } else {
        element->SetAttribute(CNS_TAG_ATTR_PF, CNS_TAG_ATTR_TRUE);
    }

    element->SetAttribute(CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps);
    element->SetAttribute(CNS_TAG_ATTR_NODESCREATED, nodescreated);
    element->SetAttribute(CNS_TAG_ATTR_SECTIONS, sections);
    element->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, length);
    element->SetAttribute(CNS_TAG_ATTR_TIME, timeValue.c_str());
    element->SetDoubleAttribute(CNS_TAG_ATTR_MAXANGLE, maxAngle);
}
