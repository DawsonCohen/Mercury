#ifndef __VOXEL_ROBOT_H__
#define __VOXEL_ROBOT_H__

// Evolvable Soft Body
#include <string>
#include "EvoDevo/Evolvables/SoftBody.h"

#define MIN_FITNESS (float) 0

namespace EvoDevo {

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
        Material mat = materials::bone;
        float radius = 0;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        float max_radius = 5.0f;
        void Randomize(float xlim, float ylim, float zlim);

        friend void swap(Circle& c1, Circle& c2) {
            using std::swap;
            swap(c1.radius, c2.radius);
            swap(c1.center, c2.center);
        }
    };

    struct Voxel {
        uint16_t ID;
        BasisIdx indices;
        Eigen::Vector3f center;
        Eigen::Vector3f base;
        Material mat = materials::air;

        void setRandomMaterial();

        std::string to_string() const {
            return std::to_string(ID) + "," +
                    std::to_string(indices.x) + "," + 
                    std::to_string(indices.y) + "," + 
                    std::to_string(indices.z) + "," +
                    std::to_string(center.x()) + "," + 
                    std::to_string(center.y()) + "," + 
                    std::to_string(center.z()) + "," +
                    std::to_string(base.x()) + "," + 
                    std::to_string(base.y()) + "," + 
                    std::to_string(base.z()) + "," +
                    mat.to_string();
        }

        friend void swap(Voxel& v1, Voxel& v2) {
            using std::swap;
            swap(v1.mat, v2.mat);
        }
    };

    class VoxelRobot : public SoftBody {
    public:
    enum Encoding {
        ENCODE_RADIUS = 0,
        ENCODE_DIRECT = 1,
        ENCODE_FUNC = 2,
        ENCODE_SHAPE_ACT = 3
    };

    private:

        static CandidatePair<VoxelRobot> TwoPointChildren(const CandidatePair<VoxelRobot>& parents);
        static CandidatePair<VoxelRobot> RadiusChildren(const CandidatePair<VoxelRobot>& parents);
        void BuildSpringsRecurse(std::vector<Spring>& springs, BasisIdx indices, std::vector<bool>& visit_list, uint16_t srcIdx = 0);
        void BuildFaces();
        void SortBoundaryMasses();
        void BuildFromCircles();
        void Initialize();

        // Gets number of non-air connected voxels from a given voxel index
        uint16_t ConnectedVoxelRecurse(BasisIdx ind, std::vector<bool>& visit_list);

        // Builds a robot R from voxels connected to a source voxel at index ind
        void CopyFromVoxelRecurse(BasisIdx ind, VoxelRobot& R, std::vector<bool>& visit_list);
        void Strip();

    private:
        float xSize = 11.0f;
        float ySize = 11.0f;
        float zSize = 11.0f;
        float resolution = 1.0f; // Masses per cm
        Eigen::Vector3f center = Eigen::Vector3f(xSize/2.0f, ySize/2.0f, 0.0f);
        std::vector<Voxel> voxels;
        std::vector<Circle> circles;
        uint16_t xCount;
        uint16_t yCount;
        uint16_t zCount;

    public:
        static Encoding repr;
        
        void Build();
        static void BatchBuild(std::vector<VoxelRobot>& robots);
        uint16_t getVoxelIdx(uint16_t xIdx, uint16_t yIdx, uint16_t zIdx) {
            return xIdx + yIdx * xCount + zIdx * (xCount*yCount);
        }

        uint16_t getVoxelIdx(BasisIdx i) {
            return i.x + i.y * xCount + i.z * (xCount*yCount);
        }

        BasisIdx getVoxelBasisIdx(uint16_t idx) {
            uint16_t xIdx = idx % xCount;
            uint16_t yIdx = (idx / xCount) % yCount;
            uint16_t zIdx = (idx / xCount / yCount) % zCount;
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
            xSize(src.xSize), ySize(src.ySize), zSize(src.zSize),
            resolution(src.resolution), voxels(src.voxels), circles(src.circles),
            xCount(src.xCount), yCount(src.yCount), zCount(src.zCount)
        { }

        VoxelRobot(const SoftBody& src) : SoftBody(src) { }

        // TODO: line method, don't assume rect.prism.
        bool isInside(Eigen::Vector3f point) {
            return 
                point.x() <= xSize && point.x() >= 0 &&
                point.y() <= ySize && point.y() >= 0 &&
                point.z() <= zSize && point.z() >= 0;
        }

        bool isValidIdx(BasisIdx ind) {
            return 
                ind.x < (int) xCount && ind.x >= 0 &&
                ind.y < (int) yCount && ind.y >= 0 &&
                ind.z < (int) zCount && ind.z >= 0;
        }

        std::vector<Voxel>& getVoxels() { return voxels; }

        void Randomize() override;
        void Mutate() override;
        void Duplicate(const VoxelRobot&);
        
        static CandidatePair<VoxelRobot> Crossover(const CandidatePair<VoxelRobot>& parents);

        friend void swap(VoxelRobot& r1, VoxelRobot& r2) {
            using std::swap;
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

        static std::vector<float> findDiversity(std::vector<VoxelRobot> pop);
        static float Distance(const CandidatePair<VoxelRobot>& robots);

    };

}

#endif
