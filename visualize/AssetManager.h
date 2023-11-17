#ifndef __ASSET_MANAGER_H__
#define __ASSET_MANAGER_H__

#include "robot_model.h"

class AssetManager {
public:
    AssetManager(std::vector<RobotModel> initAssets = std::vector<RobotModel>()) :
        mAssets(initAssets), mCurrentAssetIndex(0)
    {}

    void loadAssets(std::vector<RobotModel> newAssets) {
        mAssets.insert(mAssets.end(), newAssets.begin(), newAssets.end());
    }

    void loadAsset(RobotModel newAsset) {
        mAssets.push_back(newAsset);
    }

    void switchToNextAsset() {
        if(mCurrentAssetIndex + 1 == mAssets.size()) mAssetWrappedFlag = true;
        mCurrentAssetIndex = (mCurrentAssetIndex + 1) % mAssets.size();
        mAssetChangedFlag = true;
    }

    bool hasAssetChanged() {
        return mAssetChangedFlag;
    }

    bool hasWrapped() {
        return mAssetWrappedFlag;
    }

    size_t getAssetIndex() {
        return mCurrentAssetIndex;
    }

    void clearAssetChangedFlag() {
        mAssetChangedFlag = false;
    }

    void clearAssetWrappefdFlag() {
        mAssetWrappedFlag = false;
    }

    RobotModel getCurrentAsset() const {
        return mAssets[mCurrentAssetIndex];
    }

private:
    std::vector<RobotModel> mAssets;
    size_t mCurrentAssetIndex;
    bool mAssetChangedFlag = false;
    bool mAssetWrappedFlag = false;
};

#endif