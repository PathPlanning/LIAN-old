#include"cMap.h"
#include<string>
#include<algorithm>
#include <sstream>

cMap::cMap() {
    height = -1;
    width = -1;
    altitude_max = 0;
    min_altitude_limit = max_altitude_limit = 0;
    start_i = -1;
    start_j = -1;
    start_z = 0;
    goal_i = -1;
    goal_j = -1;
    goal_z = 0;
    Grid = nullptr;
}

cMap::~cMap() {
    if (Grid) {
        for (int i = 0; i < height; i++) {
            delete[] Grid[i];
        }

        delete[] Grid;
    }
}

bool cMap::getMap(const char *FileName) {
    const char *grid = 0;
    std::string value;
    TiXmlElement *root = 0;
    std::string text = "";
    bool hasGrid = false;
    bool hasMAXALT = false;
    bool hasALTLIM = false;
    bool hasZCoord = false;
    std::stringstream stream;
    TiXmlDocument doc(FileName);
    if (!doc.LoadFile()) {
        std::cout << "Error openning input XML file." << std::endl;
        return false;
    } else

        root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "No '" << CNS_TAG_ROOT << "' element found in XML file." << std::endl;
        return false;
    }

    TiXmlElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map) {
        std::cout << "No '" << CNS_TAG_MAP << "' element found in XML file." << std::endl;
        return false;
    }

    TiXmlNode *node = 0;
    TiXmlElement *element = 0;

    node = map->FirstChild();

    while (node) {
        if (node->Type() == TiXmlNode::TINYXML_COMMENT) {
            node = map->IterateChildren(node);
            continue;
        }
        element = node->ToElement();
        value = node->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        stream << element->GetText();

        if (!hasGrid && height > 0 && width > 0) {
            Grid = new int *[height];
            for (int i = 0; i < height; i++) {
                Grid[i] = new int[width];
            }

            hasGrid = true;
        }

        if (value == CNS_TAG_HEIGHT) {
            stream >> height;
            if (height <= 0) {
                std::cout << "Wrong '" << CNS_TAG_HEIGHT << "' value." << std::endl;
                return false;
            }
        } else if (value == CNS_TAG_WIDTH) {
            stream >> width;

            if (width <= 0) {
                std::cout << "Wrong '" << CNS_TAG_WIDTH << "' value." << std::endl;
                return false;
            }
        } else if (value == CNS_TAG_MAXALT) {
            int altitude;
            if (!((stream >> altitude) && (altitude > 0))) {
                std::cout << "Warning! Invalid value of '" << CNS_TAG_MAXALT
                          << "' tag encountered (or could not convert to integer)." << std::endl;
                std::cout << "Value of '" << CNS_TAG_MAXALT << "' tag should be an integer AND >=0" << std::endl;
                std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_MAXALT
                          << "' tag will be encountered later..." << std::endl;

            } else {
                hasMAXALT = true;
                altitude_max = std::max(altitude_max, altitude);
                if (!hasALTLIM) {
                    max_altitude_limit = altitude_max;
                }
            }
        } else if (value == CNS_TAG_ALTLIM) {
            if (hasALTLIM) {
                std::cout << "Warning! Duplicate '" << CNS_TAG_ALTLIM << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_ALTLIM << "will be used." << std::endl;
            } else {
                hasALTLIM = true;
                if (!((element->Attribute(CNS_TAG_ALTLIM_ATTR_MIN, &min_altitude_limit)) && (min_altitude_limit >= 0))) {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_ALTLIM_ATTR_MIN << "' attribute of '"
                              << CNS_TAG_ALTLIM << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_ALTLIM_ATTR_MIN << "' tag should be an integer AND >=0"
                              << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_ALTLIM_ATTR_MIN
                              << "' tag will be encountered later..." << std::endl;
                    hasALTLIM = false;
                }
                if (!((element->Attribute(CNS_TAG_ALTLIM_ATTR_MAX, &max_altitude_limit)) &&
                      (max_altitude_limit >= min_altitude_limit))) {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_ALTLIM_ATTR_MAX << "' attribute of '"
                              << CNS_TAG_ALTLIM << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_ALTLIM_ATTR_MAX
                              << "' tag should be an integer AND be not less than attribute '"
                              << CNS_TAG_ALTLIM_ATTR_MIN << "'" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_ALTLIM_ATTR_MIN
                              << "' tag will be encountered later..." << std::endl;
                    hasALTLIM = false;
                }
            }
        } else if (value == CNS_TAG_SX) {
            stream >> start_j;

            if (start_j < 0 || start_j >= width) {
                std::cout << "Wrong '" << CNS_TAG_SX << "' value." << std::endl;
                return false;
            }
        } else if (value == CNS_TAG_SY) {
            stream >> start_i;

            if (start_i < 0 || start_i >= height) {
                std::cout << "Wrong '" << CNS_TAG_SY << "' value." << std::endl;
                return false;
            }
        } else if (value == CNS_TAG_SZ) {
            if (!(stream >> start_z && start_z >= 0)) {
                std::cout << "Warning! Invalid value of '" << CNS_TAG_SZ
                          << "' tag encountered (or could not convert to integer)" << std::endl;
                std::cout << "Value of '" << CNS_TAG_SZ << "' tag should be an integer AND >=0 '"
                          << std::endl;
                std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_SZ
                          << "' tag will be encountered later..." << std::endl;
            } else {
                hasZCoord = true;
            }
        } else if (value == CNS_TAG_FX) {
            stream >> goal_j;

            if (goal_j < 0 || goal_j >= width) {
                std::cout << "Wrong '" << CNS_TAG_FX << "' value." << std::endl;
                return false;
            }
        } else if (value == CNS_TAG_FY) {
            stream >> goal_i;

            if (goal_i < 0 || goal_i >= height) {
                std::cout << "Wrong '" << CNS_TAG_FY << "' value." << std::endl;
                return false;
            }
        } else if (value == CNS_TAG_FZ) { // Finish Z
            if (!(stream >> goal_z && goal_z >= 0)) {
                std::cout << "Warning! Invalid value of '" << CNS_TAG_FZ
                          << "' tag encountered (or could not convert to integer)" << std::endl;
                std::cout << "Value of '" << CNS_TAG_FZ << "' tag should be an integer AND >=0 '"
                          << std::endl;
                std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_FZ
                          << "' tag will be encountered later..." << std::endl;
            } else {
                hasZCoord = true;
            }
        } else if (value == CNS_TAG_GRID) {
            if (height == -1 || width == -1) {
                std::cout << "No '" << CNS_TAG_HEIGHT << "' or '" << CNS_TAG_WIDTH << "' before '" << CNS_TAG_GRID
                          << "' given." << std::endl;
                return false;
            }
            stream.clear();
            stream.str("");

            element = node->FirstChildElement(CNS_TAG_ROW);

            int i = 0;
            int j, cell_value;
            while (i < height) {
                if (!element) {
                    std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
                    return false;
                }

                stream << element->GetText();
                j = 0;
                for (; j < width && (stream >> cell_value); ++j) {
                    Grid[i][j] = cell_value;
                    if (hasMAXALT || hasALTLIM || hasZCoord) {
                        altitude_max = std::max(altitude_max, cell_value);
                    }
                }

                stream.clear();
                stream.str("");

                if (j < width) {
                    std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given." << std::endl;
                    return false;
                }

                i++;
                element = element->NextSiblingElement();
            }
        }

        stream.clear();
        stream.str("");
        node = map->IterateChildren(node);
    }

    bool correct_data = true;
    if (hasZCoord && !hasALTLIM) {
        min_altitude_limit = std::min(start_z, goal_z);
        max_altitude_limit = std::max(std::max(start_z, goal_z), altitude_max);
    }

    if (start_i == -1 || start_j == -1) {
        std::cout << "Error: Couldn't find correct start point. Please, enter correct '" << CNS_TAG_SX << "' and '" <<
                  CNS_TAG_SY << "' tags.\n";
        correct_data = false;
    }

    if (Grid[start_i][start_j] > start_z) {
        std::cout << "Error: Start point is an obstacle.\n";
        correct_data = false;
    }

    if (start_z < min_altitude_limit) {
        std::cout << "Error: Start point is lower than minimum allowed altitude\n";
        correct_data = false;
    }

    if (start_z > max_altitude_limit) {
        std::cout << "Error: Start point is higher than maximum allowed altitude\n";
        correct_data = false;
    }

    if (goal_i == -1 || goal_j == -1) {
        std::cout << "Error: Couldn't find correct finish point. Please, enter correct '" << CNS_TAG_FX << "' and '" <<
                  CNS_TAG_FY << "' tags.\n";
        correct_data = false;
    }

    if (Grid[goal_i][goal_j] > goal_z) {
        std::cout << "Error: Finish point is an obstacle.\n";
        correct_data = false;
    }

    if (goal_z < min_altitude_limit) {
        std::cout << "Error: finish point is lower than minimum allowed altitude\n";
        correct_data = false;
    }

    if (goal_z > max_altitude_limit) {
        std::cout << "Error: Finish point is higher than maximum allowed altitude\n";
        correct_data = false;
    }

    if (!correct_data) {
        std::cout << "Can not continue search. Please fix the errors above and try again.\n";
        return false;
    }

    return true;
}

bool cMap::CellOnGrid(int i, int j) const {
    return i >= 0 && i < height && j >= 0 && j < width;
}

bool cMap::CellOnGrid(int i, int j, int z) const {
    return z >= min_altitude_limit && z <= max_altitude_limit && CellOnGrid(i, j);
}

bool cMap::CellIsObstacle(int i, int j) const {
    return (Grid[i][j] > 0);
}

bool cMap::CellIsTraversable(int i, int j) const {
    return (Grid[i][j] == 0);
}

int cMap::getValue(int i, int j) const {
    return Grid[i][j];
}

bool cMap::CellIsObstacle(int i, int j, int h) const {
    return (h < Grid[i][j]);
}

bool cMap::CellIsTraversable(int i, int j, int h) const {
    return (h >= Grid[i][j]);
}

bool cMap::NodeOnGrid(const Node &node) const {
    return CellOnGrid(node.i, node.j, node.z);
}

bool cMap::NodeIsTraversable(const Node &node) const {
    return CellIsTraversable(node.i, node.j, node.z);
}

bool cMap::NodeIsObstacle(const Node &node) const {
    return CellIsObstacle(node.i, node.j, node.z);
}
