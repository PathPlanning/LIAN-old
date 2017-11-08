#include"xml_logger.h"
#include"gl_const.h"
#include <sstream>

XmlLogger::XmlLogger(float loglvl) {
    loglevel = loglvl;
    LogFileName = "";
    doc = 0;
}

XmlLogger::~XmlLogger() {
    if (doc) {
        doc->Clear();
        delete doc;
    }
}

bool XmlLogger::getLog(const char *FileName) {
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

void XmlLogger::writeToLogMap(const Map &map, const std::list<Node> &path) {
    if (loglevel == CN_LOGLVL_NO) return;
    std::stringstream stream;
    std::string text, value;
    int *curLine;
    std::list<Node>::const_iterator iter;

    curLine = new int[map.width];

    for (int i = 0; i < map.width; i++)
        curLine[i] = 0;

    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_PATH);
    TiXmlElement *msg;

    for (int i = 0; i < map.height; i++) {
        msg = new TiXmlElement(CNS_TAG_ROW);
        msg->SetAttribute(CNS_TAG_ATTR_NUM, i);
        text = "";

        for (iter = path.begin(); iter != path.end(); ++iter) {
            if (iter->cell.i == i) {
                curLine[iter->cell.j] = 1;
            }
        }

        for (int j = 0; j < map.width; j++) {
            if (curLine[j] != 1) {
                stream << map[i][j];
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

void XmlLogger::writeToLogOpenClose(const std::vector<std::list<Node> > &open,
                                    const std::unordered_multimap<int, Node> &close) {

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

    // Deprecated. Writing out minimum node
    /*
    Node min;
    min.F = -1;
    int exc = 0;
    for (int i = 0; i < open.size(); i++)
        if (!open[i].empty())
            if (open[i].begin()->F <= min.F || min.F == -1) {
                if (open[i].List.begin()->F == min.F && open[i].List.begin()->g > min.g) {
                    min = *open[i].List.begin();
                    exc = i;
                } else if (open[i].List.begin()->F < min.F || min.F == -1) {
                    min = *open[i].List.begin();
                    exc = i;
                }
            }
    if (min.F != -1) {
        element = new TiXmlElement(CNS_TAG_NODE);
        element->SetAttribute(CNS_TAG_ATTR_X, min.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element->SetDoubleAttribute(CNS_TAG_ATTR_F, min.F);
        element->SetDoubleAttribute(CNS_TAG_ATTR_G, min.g);
        element->SetAttribute(CNS_TAG_ATTR_PARX, min.Parent->j);
        element->SetAttribute(CNS_TAG_ATTR_PARY, min.Parent->i);
        child->InsertEndChild(*element);
    }
     */
    for (int i = 0; i < open.size(); i++)
        if (!open[i].empty())
            for (auto it = open[i].begin(); it != open[i].end(); ++it) {
                element->Clear();
                element->SetAttribute(CNS_TAG_ATTR_X, it->cell.j);
                element->SetAttribute(CNS_TAG_ATTR_Y, it->cell.i);
                element->SetDoubleAttribute(CNS_TAG_ATTR_F, it->F);
                element->SetDoubleAttribute(CNS_TAG_ATTR_G, it->g);
                if (it->g > 0) {
                    element->SetAttribute(CNS_TAG_ATTR_PARX, it->parent->cell.j);
                    element->SetAttribute(CNS_TAG_ATTR_PARY, it->parent->cell.i);
                }
                child->InsertEndChild(*element);
            }

    element = new TiXmlElement(CNS_TAG_CLOSE);
    lowlevel->InsertEndChild(*element);
    child = lowlevel->LastChild();

    for (std::unordered_map<int, Node>::const_iterator it = close.begin(); it != close.end(); ++it) {
        element = new TiXmlElement(CNS_TAG_NODE);
        element->SetAttribute(CNS_TAG_ATTR_X, it->second.cell.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->second.cell.i);
        element->SetDoubleAttribute(CNS_TAG_ATTR_F, it->second.F);
        element->SetDoubleAttribute(CNS_TAG_ATTR_G, it->second.g);
        if (it->second.g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, it->second.parent->cell.j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, it->second.parent->cell.i);
        }
        child->InsertEndChild(*element);
    }
}

void XmlLogger::writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles) {
    if (loglevel == CN_LOGLVL_NO) return;
    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_LPLEVEL);

    TiXmlElement *point;

    std::list<Node>::const_iterator iter;
    int i = 0;

    for (iter = path.begin(); iter != path.end(); ++iter) {
        point = new TiXmlElement(CNS_TAG_NODE);
        point->SetAttribute(CNS_TAG_ATTR_NUM, i);
        point->SetAttribute(CNS_TAG_ATTR_X, iter->cell.j);
        point->SetAttribute(CNS_TAG_ATTR_Y, iter->cell.i);
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

void XmlLogger::saveLog() {
    if (loglevel == CN_LOGLVL_NO) return;
    doc->SaveFile(LogFileName.c_str());
}

void XmlLogger::writeToLogHpLevel(const std::list<Node> &path) {
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
        part->SetAttribute(CNS_TAG_ATTR_SX, it->cell.j);
        part->SetAttribute(CNS_TAG_ATTR_SY, it->cell.i);
        iter++;
        part->SetAttribute(CNS_TAG_ATTR_FX, iter->cell.j);
        part->SetAttribute(CNS_TAG_ATTR_FY, iter->cell.i);
        part->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
        hplevel->LinkEndChild(part);
        it++;
        partnumber++;
    }
}

void XmlLogger::writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated,
                                  float length, double Time, float maxAngle, int sections) {
    if (loglevel == CN_LOGLVL_NO) return;
    TiXmlElement *element = doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element = element->FirstChildElement(CNS_TAG_SUM);

    if (path.size() == 0) {
        element->SetAttribute(CNS_TAG_ATTR_PF, CNS_TAG_ATTR_FALSE);
    } else {
        element->SetAttribute(CNS_TAG_ATTR_PF, CNS_TAG_ATTR_TRUE);
    }

    element->SetAttribute(CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps);
    element->SetAttribute(CNS_TAG_ATTR_NODESCREATED, nodescreated);
    element->SetAttribute(CNS_TAG_ATTR_SECTIONS, sections);
    element->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, length);
    element->SetDoubleAttribute(CNS_TAG_ATTR_TIME, Time);
    element->SetDoubleAttribute(CNS_TAG_ATTR_MAXANGLE, maxAngle);
}
