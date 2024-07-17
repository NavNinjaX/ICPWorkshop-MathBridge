#include "PLYReader.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<Point> PLYReader::read(const std::string& filepath) {
    std::vector<Point> points;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return points;
    }

    std::string line;
    bool header = true;

    while (std::getline(file, line)) {
        if (header) {
            if (line == "end_header") {
                header = false;
            }
            continue;
        }

        std::istringstream iss(line);
        Point point;
        iss >> point.x >> point.y >> point.z;
        points.push_back(point);
    }

    file.close();
    return points;
}
