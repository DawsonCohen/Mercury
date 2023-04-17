#ifndef __KNN_H__
#define __KNN_H__

#include <vector>
#include "NNRobot.h"

namespace KNN {

    /// @brief Computes the K Nearest neighbors for each mass of each mass_list
    /// @tparam T - requires variable masses
    /// @param point_groups 
    /// @param k 
    /// @return vector of KNNs for each mass in each group
    template<typename T>
    std::vector<std::vector<std::vector<std::pair<unsigned int,float>>>> Batch(const std::vector<T>& mass_groups, unsigned int K);
};

#endif
