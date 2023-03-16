#pragma once

#include <memory>
#include <unordered_map>
#include <fstream>
#include <vector>
#include "ghost_ros/helpers/pathing/Path.h"
#include "ghost_ros/helpers/pathing/PathPoint.h"
#include "ghost_ros/helpers/3rdparty/json.hpp"
#include "ghost_ros/helpers/math/Math.h"
#include "ghost_ros/helpers/util/Logger.h"

using namespace nlohmann;

class PathManager {
public:
    static PathManager* GetInstance();

    bool LoadPathsText(std::string text);
    bool LoadPaths(json pathJson);
    bool LoadPathsFile(std::string filePath);

    int NumPaths();

    std::unordered_map<std::string, Path> GetPaths();

    Path GetPath(std::string name);

private:
    PathManager() = default;
    std::unordered_map<std::string, Path> m_paths;
    static PathManager* m_instance;
};