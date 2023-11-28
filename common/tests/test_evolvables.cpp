#include "SoftBody.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <string>
#include "util.h"

#include "common_tests.h"

#define EPS 0

int TestNNRobot() {
    Config::NNRobot nnConfig;
    nnConfig.massCount = 100;
    
    NNRobot::Configure(nnConfig);

    NNRobot R;
    R.Randomize();
    R.Build();

    std::string encoding = R.Encode();

    util::WriteCSV("test_NNBot.csv","./z_results/",encoding);

    NNRobot Rdecoded;
    Rdecoded.Decode("./z_results/test_NNBot.csv");

    if(R.masses.size() != Rdecoded.masses.size()) return 1;

    for(size_t i = 0; i < R.masses.size(); i++) {
        if((R.masses[i].pos - Rdecoded.masses[i].pos).norm() > EPS) {
            Eigen::Vector3f pos1 = R.masses[i].pos;
            Eigen::Vector3f pos2 = Rdecoded.masses[i].pos;
            printf("(%f,%f,%f) vs (%f,%f,%f)\n",pos1[0],pos1[1],pos1[2],pos2[0],pos2[1],pos2[2]);
            return 2;
        }
        if(R.masses[i].material != Rdecoded.masses[i].material) {
            printf("%lu: %u vs %u\n",i, R.masses[i].material.encoding, Rdecoded.masses[i].material.encoding);
            return 3;
        }
    }

    if(R.springs.size() != Rdecoded.springs.size()) return 4;

    for(size_t i = 0; i < R.springs.size(); i++) {
        if(R.springs[i].m0 != Rdecoded.springs[i].m0) return 5;
        if(R.springs[i].m1 != Rdecoded.springs[i].m1) return 6;
        if(abs(R.springs[i].rest_length - Rdecoded.springs[i].rest_length) > EPS) return 7;
        if(abs(R.springs[i].mean_length - Rdecoded.springs[i].mean_length) > EPS) return 8;
        if(R.springs[i].material != Rdecoded.springs[i].material) return 9;
    }

    return 0;
}
