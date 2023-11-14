#ifndef __ASSET_MANAGER_H__
#define __ASSET_MANAGER_H__

#include "robot_model.h"

class AssetManager {
public:
    AssetManager(std::vector<RobotModel> initAssets = std::vector<RobotModel>()) :
        assets(initAssets), currentAssetIndex(0)
    {}

    void loadAssets(std::vector<RobotModel> newAssets) {
        assets.insert(assets.end(), newAssets.begin(), newAssets.end());
    }

    void loadAsset(RobotModel newAsset) {
        assets.push_back(newAsset);
    }

    void switchToNextAsset() {
        currentAssetIndex = (currentAssetIndex + 1) % assets.size();
        assetChangedFlag = true;
    }

    bool hasAssetChanged() {
        return assetChangedFlag;
    }

    void clearAssetChangedFlag() {
        assetChangedFlag = false;
    }

    RobotModel getCurrentAsset() const {
        return assets[currentAssetIndex];
    }

private:
    std::vector<RobotModel> assets;
    int currentAssetIndex;
    bool assetChangedFlag = false;
};

#endif