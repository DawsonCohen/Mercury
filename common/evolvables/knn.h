#ifndef __KNN_H__
#define __KNN_H__

#include <vector>
#include "NNRobot.h"

namespace KNN {
    template<typename T>
    std::vector<std::vector<std::pair<uint16_t,float>>> KNN(const T& mass_group, uint16_t K);

    template<typename T>
    std::vector<std::vector<std::pair<uint16_t,float>>> KNN_CPU(const T& mass_group, uint16_t K);

    /// @brief Computes the K Nearest neighbors for each mass of each mass_list
    /// @tparam T - requires variable masses
    /// @param point_groups 
    /// @param k 
    /// @return vector of KNNs for each mass in each group
    template<typename T>
    std::vector<std::vector<std::vector<std::pair<uint16_t,float>>>> Batch(const std::vector<T>& mass_groups, uint16_t K);
}

#endif
