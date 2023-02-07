#ifndef __VOXEL_ROBOT_H__
#define __VOXEL_ROBOT_H__

// Evolvable Soft Body
#include <random>
#include <string>
#include "SoftBody.h"

#define MIN_FITNESS (float) 0

struct VoxelRobotPair;

struct BasisIdx {
    int x;
    int y;
    int z;

    BasisIdx operator+(const BasisIdx& src) {
        return {x + src.x, y + src.y, z + src.z};
    }

    BasisIdx operator-(const BasisIdx& src) {
        return {x - src.x, y - src.y, z - src.z};
    }

    bool operator==(const BasisIdx& src) {
        return {x == src.x && y == src.y && z == src.z};
    }
};

struct Circle {
    MaterialOption mat = BONE;
    float radius = 0;
    glm::vec3 center = glm::vec3(0.0f);
    float max_radius = 5.0f;
    void Randomize(float xlim, float ylim, float zlim);

    friend void swap(Circle& c1, Circle& c2) {
        using std::swap;
        swap(c1.radius, c2.radius);
        swap(c1.center, c2.center);
    }
};

struct Voxel {
    uint ID;
    BasisIdx indices;
    glm::vec3 center;
    glm::vec3 base;
    Material mat;

    void setRandomMaterial() {
        mat = materials::bone;
    }

    std::string to_string() const {
        return std::to_string(ID) + "," +
                std::to_string(indices.x) + "," + 
                std::to_string(indices.y) + "," + 
                std::to_string(indices.z) + "," +
                std::to_string(center.x) + "," + 
                std::to_string(center.y) + "," + 
                std::to_string(center.z) + "," +
                std::to_string(base.x) + "," + 
                std::to_string(base.y) + "," + 
                std::to_string(base.z) + "," +
                mat.to_string();
    }

    friend void swap(Voxel& v1, Voxel& v2) {
        using std::swap;
        swap(v1.mat, v2.mat);
    }
};

class VoxelRobot : public SoftBody {
friend class Evaluator;

public:
enum Encoding {
    ENCODE_DIRECT = 0,
    ENCODE_RADIUS = 1,
    ENCODE_FUNC = 2,
    ENCODE_SHAPE_ACT = 3
};


private:
    float   mFitness = 0;
    uint    mVolume = 0;
    uint    mParetoLayer = 0;

public:
    int     mAge = 0;
    bool wasUsed = false;

    glm::vec3 mCOM;
    glm::vec3 mSkew;

    static unsigned seed;
    static std::default_random_engine gen;
    static std::uniform_real_distribution<> uniform;

    float Distance();

    static VoxelRobotPair TwoPointChildren(const VoxelRobotPair& parents);
    static VoxelRobotPair RadiusChildren(const VoxelRobotPair& parents);
    void BuildSpringsRecurse(std::vector<Spring>& springs, BasisIdx indices, bool* visit_list, uint srcIdx = 0);
    void BuildFromCircles();
    void Build();
    void Initialize();

    void CopyFromVoxelRecurse(BasisIdx ind, VoxelRobot& R, std::vector<bool>& visit_list);
    uint StripRecurse(BasisIdx ind, std::vector<bool>& visit_list);
    void Strip();

private:
    float xSize = 10.0f;
    float ySize = 10.0f;
    float zSize = 10.0f;
    float resolution = 1.0f; // Masses per meter
    glm::vec3 center = glm::vec3(xSize/2, ySize/2, 0);
    std::vector<Voxel> voxels;
    std::vector<Circle> circles;
    uint xCount;
    uint yCount;
    uint zCount;

public:
    inline static Encoding repr;
    
    uint getVoxelIdx(uint xIdx, uint yIdx, uint zIdx) {
        return xIdx + yIdx * xCount + zIdx * (xCount*yCount);
    }

    uint getVoxelIdx(BasisIdx i) {
        return i.x + i.y * xCount + i.z * (xCount*yCount);
    }

    BasisIdx getVoxelBasisIdx(uint idx) {
        uint xIdx = idx % xCount;
        uint yIdx = (idx / xCount) % yCount;
        uint zIdx = (idx / xCount / yCount) % zCount;
        return {(int) xIdx, (int) yIdx, (int) zIdx};
    }

    void setUniformSize(float sz) {
        setXSize(sz);
        setYSize(sz);
        setZSize(sz);
    }

    void setXSize(float sz) { 
        xSize = sz;
        xCount = sz * resolution;
    }
    void setYSize(float sz) { 
        ySize = sz;
        yCount = sz * resolution;
    }
    void setZSize(float sz) { 
        zSize = sz;
        zCount = sz * resolution;
    }

    void setResolution(float res) {
        resolution = res;
        xCount = xSize * res;
        yCount = ySize * res;
        zCount = zSize * res;
    }

    VoxelRobot();
    VoxelRobot(float x, float y, float z, float res, std::vector<Voxel> voxels) :
        xSize(x), ySize(y), zSize(z), resolution(res), voxels(voxels)
    {
        xCount = resolution*xSize;
        yCount = resolution*ySize;
        zCount = resolution*zSize;
        Build();
    };
    
    VoxelRobot(const VoxelRobot& src) : SoftBody(src),
        mFitness(src.mFitness), mVolume(src.mVolume), 
        mParetoLayer(src.mParetoLayer),mAge(src.mAge), mCOM(src.mCOM), mSkew(src.mSkew),
        xSize(src.xSize), ySize(src.ySize), zSize(src.zSize),
        resolution(src.resolution), voxels(src.voxels), circles(src.circles),
        xCount(src.xCount), yCount(src.yCount), zCount(src.zCount)
    { }

    VoxelRobot(const SoftBody& src) : SoftBody(src) { }

    // TODO: line method, don't assume rect.prism.
    bool isInside(glm::vec3 point) {
        return 
            point.x <= xSize && point.x >= 0 &&
            point.y <= ySize && point.y >= 0 &&
            point.z <= zSize && point.z >= 0;
    }

    bool isValidInd(BasisIdx ind) {
        return 
            ind.x < (int) xCount && ind.x >= 0 &&
            ind.y < (int) yCount && ind.y >= 0 &&
            ind.z < (int) zCount && ind.z >= 0;
    }

    std::vector<Voxel>& getVoxels() { return voxels; }

    float fitness() const { return mFitness; }
    uint volume() const { return mVolume; }
    glm::vec3 COM() const { return mCOM; }
    glm::vec3 skew() const { return mSkew; }
    uint paretoLayer() const { return mParetoLayer; }
    void setFitness(float fit) { mFitness = fit; }

    void IncrementAge() { mAge++; };
    void Randomize();
    static void Random();
    void Duplicate(const VoxelRobot&);
    
    void Mutate();
    static VoxelRobotPair Crossover(const VoxelRobotPair& parents);

    static glm::vec3 calcMeanPos(VoxelRobot&);
    static glm::vec3 calcSkew(VoxelRobot&);
    static void calcFitness(VoxelRobot&);
    static float Distance(const VoxelRobotPair& robots);

    std::string DirectEncode() const;

    std::string Encode() const;

    friend void swap(VoxelRobot& r1, VoxelRobot& r2) {
        using std::swap;
        swap(r1.mFitness, r2.mFitness);
        swap(r1.mVolume, r2.mVolume);
        swap(r1.mAge, r2.mAge);
        swap(r1.mParetoLayer, r2.mParetoLayer);
        swap(r1.mCOM, r2.mCOM);
        swap(r1.mSkew, r2.mSkew);
        swap(r1.voxels, r2.voxels);
        swap(r1.circles, r2.circles);
        swap(r1.xSize, r2.xSize);
        swap(r1.ySize, r2.ySize);
        swap(r1.zSize, r2.zSize);
        swap(r1.resolution, r2.resolution);
        swap(r1.xCount, r2.xCount);
        swap(r1.yCount, r2.yCount);
        swap(r1.zCount, r2.zCount);

        swap((SoftBody&) r1, (SoftBody&) r2);
    }

    VoxelRobot& operator=(VoxelRobot src) {
        swap(*this, src);

        return *this;
    }

    bool operator < (const VoxelRobot& R) const {
        return mParetoLayer > R.mParetoLayer;
    }

    bool operator > (const VoxelRobot& R) const {
        return mParetoLayer < R.mParetoLayer;
    }

    bool operator <= (const VoxelRobot& R) const {
        return !(mFitness > R.mFitness);
    }

    bool operator >= (const VoxelRobot& R) const {
        return !(mFitness < R.mFitness);
    }

    static std::vector<float> findDiversity(std::vector<VoxelRobot> pop);
};

struct VoxelRobotPair {
    VoxelRobot first;
    VoxelRobot second;

    VoxelRobotPair(const VoxelRobot _first, const VoxelRobot _second) : 
        first(_first), second(_second) {}
    
    VoxelRobotPair() {}
};

#endif
