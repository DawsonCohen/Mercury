#include <fstream>
#include <iostream>
#include <memory>
#include <dirent.h>
#include <unistd.h>
#include <string>
#include "util.h"

namespace util {
    
VoxelRobot ReadVoxelRobot(const char* filename) {
    std::ifstream file(filename);
    std::string line;
    
    std::vector<Voxel> voxels;

    uint ID;
    int xIdx, yIdx, zIdx;
    float   sizeX, sizeY, sizeZ, resolution,
            cx,cy,cz,
            bx, by, bz;
    Material mat;

    std::getline(file, line, '\n');
    sizeX = atof(line.data());
    std::getline(file, line, '\n');
    sizeY = atof(line.data());
    std::getline(file, line, '\n');
    sizeZ = atof(line.data());
    std::getline(file, line, '\n');
    resolution = atof(line.data());
    
    while(file.peek() == 'v') {
        std::getline(file, line, 'v');
        std::getline(file, line, ',');
        ID = atoi(line.data());
        std::getline(file, line, ',');
        xIdx = atoi(line.data());
        std::getline(file, line, ',');
        yIdx = atoi(line.data());
        std::getline(file, line, ',');
        zIdx = atoi(line.data());
        std::getline(file, line, ',');
        cx = atof(line.data());
        std::getline(file, line, ',');
        cy = atof(line.data());
        std::getline(file, line, ',');
        cz = atof(line.data());
        std::getline(file, line, ',');
        bx = atof(line.data());
        std::getline(file, line, ',');
        by = atof(line.data());
        std::getline(file, line, ',');
        bz = atof(line.data());
        std::getline(file, line, ',');
        mat.k = atof(line.data());
        std::getline(file, line, ',');
        mat.dL0 = atof(line.data());
        std::getline(file, line, ',');
        mat.omega = atof(line.data());
        std::getline(file, line, ',');
        mat.phi = atof(line.data());
        std::getline(file, line, ',');
        mat.color.r = atof(line.data());
        std::getline(file, line, ',');
        mat.color.g = atof(line.data());
        std::getline(file, line, ',');
        mat.color.b = atof(line.data());
        std::getline(file, line, '\n');
        mat.color.a = atof(line.data());


        Eigen::Vector3f center(cx,cy,cz);
        Eigen::Vector3f base(bx,by,bz);

        Voxel v{ID, {xIdx,yIdx,zIdx}, center, base, mat};
        voxels.push_back(v);
    }
    
    VoxelRobot R(sizeX, sizeY, sizeZ, resolution, voxels);

    return R;
}

std::string FitnessHistoryToCSV(std::vector<std::tuple<ulong,float>>& h) {
    std::string s = "evaluation, solution_fitness\n";
    for(size_t i = 0; i < h.size(); i++) {
        s += std::to_string(std::get<0>(h[i])) + ", " + std::to_string(std::get<1>(h[i]))+"\n";
    }

    return s;
}

std::string PopulationFitnessHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h) {
    std::string s = "evaluation";
    for(size_t i = 0; i < std::get<1>(h[0]).size(); i++) {
        s += ", organism_"+std::to_string(i);
    }
    s+="\n";

    for(size_t i = 0; i < h.size(); i++) {
        for(size_t j = 0; j < std::get<1>(h[0]).size(); j++) {
            s += std::to_string(std::get<0>(h[i])) + ", "+std::to_string(std::get<1>(h[i])[j]);
        }
        s+="\n";
    }
    return s;
}

std::string PopulationDiversityHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h) {
    std::string s = "evaluation";
    for(size_t i = 0; i < std::get<1>(h[0]).size(); i++) {
        s += ", organism_"+std::to_string(i);
    }
    s+="\n";

    for(size_t i = 0; i < h.size(); i++) {
        for(size_t j = 0; j < std::get<1>(h[0]).size(); j++) {
            s += std::to_string(std::get<0>(h[i])) + ", "+std::to_string(std::get<2>(h[i])[j]);
        }
        s+="\n";
    }
    return s;
}


int WriteCSV(const char* filename, std::string datastring) {
    FILE *fp = fopen(filename,"w");

    if(fp)
        fwrite(datastring.data(), sizeof(char), datastring.size(), fp);
    else {
        printf("Failed to write\n");
        return 1;
    }

    fclose(fp);
    
    return 0;
}

void RemoveOldFiles(const char* dir) {
    DIR* dirp = opendir(dir);
    if (!dirp) {
        return;
    }

    struct dirent* dp;
    while ((dp = readdir(dirp)) != nullptr) {
        if (dp->d_type == DT_REG) {
            std::string filename = std::string(dir) + "/" + dp->d_name;
            unlink(filename.c_str());
        }
    }

    closedir(dirp);
}

}