#ifndef PLYREADER_H
#define PLYREADER_H

#include <vector>
#include <string>
#include "Point.h"

class PLYReader {
public:
    static std::vector<Point> read(const std::string& filepath);
};

#endif // PLYREADER_H
