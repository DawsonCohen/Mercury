#ifndef __KNN_H__
#define __KNN_H__

#include <vector>
#include "NNRobot.h"

namespace KNN {
    std::vector<std::vector<std::pair<uint16_t,float>>> KNN(const std::vector<Mass>& masses, uint16_t K);
    std::vector<std::vector<std::pair<uint16_t,float>>> KNN_CPU(const std::vector<Mass>& mass_group, uint16_t K);

    /// @brief Computes the K Nearest neighbors for each mass of each mass_list
    /// @tparam T - requires variable masses
    /// @param point_groups 
    /// @param k 
    /// @return vector of KNNs for each mass in each group
    std::vector<std::vector<std::vector<std::pair<uint16_t,float>>>> Batch(const std::vector<Mass>& mass_groups, uint16_t K);
}

#endif
