#include <map>
#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include "VoxelRobot.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

namespace EvoDevo {

    VoxelRobot::Encoding VoxelRobot::repr = VoxelRobot::ENCODE_RADIUS;


    void Voxel::setRandomMaterial() {
        mat = materials::random();
    }

    void Circle::Randomize(float xlim, float ylim, float zlim) {
        static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        static std::default_random_engine gen = std::default_random_engine(seed);
        static std::uniform_real_distribution<> uniform = std::uniform_real_distribution<>(0.0,1.0);
        
        Eigen::Vector3f new_center;
        new_center.x() = xlim*uniform(gen);
        new_center.y() = ylim*uniform(gen);
        new_center.z() = zlim*uniform(gen);
        center = new_center;
        radius = max_radius * uniform(gen);
    }

    // Note mMasses.size == voxels.size
    // Builds spring from srcIdx to vIdx
    void VoxelRobot::BuildSpringsRecurse(std::vector<Spring>& _springs, BasisIdx indices, std::vector<bool>& visit_list, uint16_t srcIdx) {
        if(!isValidIdx(indices)) return;

        uint16_t vIdx = getVoxelIdx(indices);

        Voxel v = voxels[vIdx];
        Voxel v_src = voxels[srcIdx];

        if(srcIdx != vIdx) {
            // Spring mean length
            float L = (v_src.base - v.base).norm();
            
            //  Get material of all owning voxels
            BasisIdx path = indices - v_src.indices;
            Material mat, avgMat;
            uint32_t matEncoding = 0x00u;

            BasisIdx neighbors[4];
            for(uint8_t i = 0; i < 4; i++) {
                neighbors[i] = {0,0,0};
            }
            if(abs(path.x)+abs(path.y)+abs(path.z) == 1) { // cardinal
                if(abs(path.x) == 1) {
                    for(int i = 0; i < 4; i++) {
                        BasisIdx nprox = {0,(i/2)%2,i%2}; 
                        neighbors[i] = v_src.indices - nprox;
                    }
                } else if (abs(path.y) == 1) {
                    for(int i = 0; i < 4; i++) {
                        BasisIdx nprox = {(i/2)%2,0,i%2}; 
                        neighbors[i] = v_src.indices - nprox;
                    }
                } else if (abs(path.z) == 1) {
                    for(int i = 0; i < 4; i++) {
                        BasisIdx nprox = {(i/2)%2,i%2,0};
                        neighbors[i] = v_src.indices - nprox;
                    }
                }
                for(uint8_t i = 0; i < 4; i++) {
                    if(!isValidIdx(neighbors[i])) continue;
                    Voxel neighbor = voxels[getVoxelIdx(neighbors[i])];
                    if(neighbor.mat.encoding != materials::air.encoding)
                        matEncoding |= neighbor.mat.encoding;
                }
                mat = materials::decode(matEncoding);
            } else if(abs(path.x)+abs(path.y)+abs(path.z) == 2) { // diagonal
                BasisIdx nprox = {!path.x,!path.y,!path.z};
                BasisIdx neighborIdx = v_src.indices - nprox;
                if(path.x >= 0 && path.y >= 0 && path.z >= 0 ){
                    if(!isValidIdx(neighborIdx)) {
                        mat = v_src.mat;
                    } else {
                        Voxel neighbor = voxels[getVoxelIdx(neighborIdx)];
                        if(neighbor.mat.encoding != materials::air.encoding)
                            matEncoding |= neighbor.mat.encoding;
                        if(v_src.mat.encoding != materials::air.encoding)
                            matEncoding |= v_src.mat.encoding;

                        mat = materials::decode(matEncoding);
                    }
                } else {
                    BasisIdx srcprx = {path.x==-1,path.y==-1,path.z==-1};
                    BasisIdx matSrcIdx = v_src.indices - srcprx;
                    Voxel v_matsrc = voxels[getVoxelIdx(matSrcIdx)];
                    BasisIdx neighborIdx = matSrcIdx - nprox;

                    if(!isValidIdx(neighborIdx)) {
                        mat = v_matsrc.mat;
                    } else {
                        Voxel neighbor = voxels[getVoxelIdx(neighborIdx)];
                        if(neighbor.mat.encoding != materials::air.encoding)
                            matEncoding |= neighbor.mat.encoding;
                        if(v_matsrc.mat.encoding != materials::air.encoding)
                            matEncoding |= v_matsrc.mat.encoding;
                        
                        mat = materials::decode(matEncoding);
                    }
                }
            } else { // body diagonal
                if(path.x >= 0 && path.y >= 0 && path.z >= 0 ){
                    mat = v_src.mat;
                } else { // backtrack body diagonals
                    BasisIdx srcprx = {path.x==-1,path.y==-1,path.z==-1};
                    BasisIdx matSrcIdx = v_src.indices - srcprx;
                    Voxel v_matsrc = voxels[getVoxelIdx(matSrcIdx)];
                    mat = v_matsrc.mat;
                }
            }

            Spring s{(uint16_t) srcIdx,(uint16_t) vIdx,L,L,mat};
            _springs.push_back(s);
        }


        if(visit_list[vIdx]) return;
        visit_list[vIdx] = true;
        //  Adds the following to the basis indices:
        //  001, 010, 011, 100, 101, 110, 111
        uint16_t x,y,z;
        uint16_t tempx,tempy;
        for(uint8_t k = 1; k < 8; k++) {
            x = k%2;
            y = (k/2)%2;
            z = (k/4)%2;
            BasisIdx offset = {x,y,z};
            BuildSpringsRecurse(_springs, indices + offset, visit_list, vIdx);
        }

        //  Adds the following to the basis indices:
        // 1-10, 01-1,-101
        x = 1; y = -1; z = 0;
        for(uint8_t k = 0; k < 3; k++) {
            BasisIdx offset = {x,y,z};
            BuildSpringsRecurse(_springs, indices + offset, visit_list, vIdx);
            tempx = x;
            tempy = y;
            x = z;
            y = tempx;
            z = tempy;
        }

        //  Adds the following to the basis indices:
        // -111,1-11,11-1
        x = -1; y = 1; z = 1;
        for(uint8_t k = 0; k < 3; k++) {
            BasisIdx offset = {x,y,z};
            BuildSpringsRecurse(_springs, indices + offset, visit_list, vIdx);
            tempx = x;
            tempy = y;
            x = z;
            y = tempx;
            z = tempy;
        }
    }

    void VoxelRobot::BuildFaces() {
        
        BasisIdx neighbor_idx;
        Face f1, f2;
        for(Voxel& v : voxels) {
            if(v.mat == materials::air) continue;

            // RIGHT
            neighbor_idx = v.indices + BasisIdx{1,0,0};
            if(!isValidIdx(neighbor_idx) ||
                voxels[getVoxelIdx(neighbor_idx)].mat == materials::air
            ) {
                
                f1.m0 = getVoxelIdx(v.indices + BasisIdx{1,0,0});
                f1.m1 = getVoxelIdx(v.indices + BasisIdx{1,1,1});
                f1.m2 = getVoxelIdx(v.indices + BasisIdx{1,0,1});

                f2.m0 = getVoxelIdx(v.indices + BasisIdx{1,0,0});
                f2.m1 = getVoxelIdx(v.indices + BasisIdx{1,1,1});
                f2.m2 = getVoxelIdx(v.indices + BasisIdx{1,1,0});

                masses[f1.m0].isOnBoundary = true;
                masses[f1.m1].isOnBoundary = true;
                masses[f1.m2].isOnBoundary = true;
                masses[f2.m0].isOnBoundary = true;
                masses[f2.m1].isOnBoundary = true;
                masses[f2.m2].isOnBoundary = true;

                faces.push_back(f1);
                faces.push_back(f2);
            }

            // LEFT
            neighbor_idx = v.indices + BasisIdx{-1,0,0};
            if(!isValidIdx(neighbor_idx) ||
                voxels[getVoxelIdx(neighbor_idx)].mat == materials::air ) {
                
                f1.m0 = getVoxelIdx(v.indices + BasisIdx{0,0,0});
                f1.m1 = getVoxelIdx(v.indices + BasisIdx{0,1,1});
                f1.m2 = getVoxelIdx(v.indices + BasisIdx{0,0,1});

                f2.m0 = getVoxelIdx(v.indices + BasisIdx{0,0,0});
                f2.m1 = getVoxelIdx(v.indices + BasisIdx{0,1,1});
                f2.m2 = getVoxelIdx(v.indices + BasisIdx{0,1,0});

                masses[f1.m0].isOnBoundary = true;
                masses[f1.m1].isOnBoundary = true;
                masses[f1.m2].isOnBoundary = true;
                masses[f2.m0].isOnBoundary = true;
                masses[f2.m1].isOnBoundary = true;
                masses[f2.m2].isOnBoundary = true;

                faces.push_back(f1);
                faces.push_back(f2);
            }

            // UP
            neighbor_idx = v.indices + BasisIdx{0,1,0};
            if(!isValidIdx(neighbor_idx) ||
                voxels[getVoxelIdx(neighbor_idx)].mat == materials::air ) {
                
                f1.m0 = getVoxelIdx(v.indices + BasisIdx{0,1,0});
                f1.m1 = getVoxelIdx(v.indices + BasisIdx{0,1,1});
                f1.m2 = getVoxelIdx(v.indices + BasisIdx{1,1,1});

                f2.m0 = getVoxelIdx(v.indices + BasisIdx{0,1,0});
                f2.m1 = getVoxelIdx(v.indices + BasisIdx{1,1,1});
                f2.m2 = getVoxelIdx(v.indices + BasisIdx{1,1,0});

                masses[f1.m0].isOnBoundary = true;
                masses[f1.m1].isOnBoundary = true;
                masses[f1.m2].isOnBoundary = true;
                masses[f2.m0].isOnBoundary = true;
                masses[f2.m1].isOnBoundary = true;
                masses[f2.m2].isOnBoundary = true;

                faces.push_back(f1);
                faces.push_back(f2);
            }

            // DOWN
            neighbor_idx = v.indices + BasisIdx{0,-1,0};
            if(!isValidIdx(neighbor_idx) ||
                voxels[getVoxelIdx(neighbor_idx)].mat == materials::air ) {
                
                f1.m0 = getVoxelIdx(v.indices + BasisIdx{0,0,0});
                f1.m1 = getVoxelIdx(v.indices + BasisIdx{0,0,1});
                f1.m2 = getVoxelIdx(v.indices + BasisIdx{1,0,1});

                f2.m0 = getVoxelIdx(v.indices + BasisIdx{0,0,0});
                f2.m1 = getVoxelIdx(v.indices + BasisIdx{1,0,1});
                f2.m2 = getVoxelIdx(v.indices + BasisIdx{1,0,0});

                masses[f1.m0].isOnBoundary = true;
                masses[f1.m1].isOnBoundary = true;
                masses[f1.m2].isOnBoundary = true;
                masses[f2.m0].isOnBoundary = true;
                masses[f2.m1].isOnBoundary = true;
                masses[f2.m2].isOnBoundary = true;

                faces.push_back(f1);
                faces.push_back(f2);
            }

            // FORWARD
            neighbor_idx = v.indices + BasisIdx{0,0,1};
            if(!isValidIdx(neighbor_idx) ||
                voxels[getVoxelIdx(neighbor_idx)].mat == materials::air ) {
                
                f1.m0 = getVoxelIdx(v.indices + BasisIdx{0,0,1});
                f1.m1 = getVoxelIdx(v.indices + BasisIdx{1,0,1});
                f1.m2 = getVoxelIdx(v.indices + BasisIdx{0,1,1});

                f2.m0 = getVoxelIdx(v.indices + BasisIdx{1,0,1});
                f2.m1 = getVoxelIdx(v.indices + BasisIdx{1,1,1});
                f2.m2 = getVoxelIdx(v.indices + BasisIdx{0,1,1});

                masses[f1.m0].isOnBoundary = true;
                masses[f1.m1].isOnBoundary = true;
                masses[f1.m2].isOnBoundary = true;
                masses[f2.m0].isOnBoundary = true;
                masses[f2.m1].isOnBoundary = true;
                masses[f2.m2].isOnBoundary = true;

                faces.push_back(f1);
                faces.push_back(f2);
            }

            // BACK
            neighbor_idx = v.indices + BasisIdx{0,0,-1};
            if(!isValidIdx(neighbor_idx) ||
                voxels[getVoxelIdx(neighbor_idx)].mat == materials::air ) {
                
                f1.m0 = getVoxelIdx(v.indices + BasisIdx{0,0,1});
                f1.m1 = getVoxelIdx(v.indices + BasisIdx{1,0,1});
                f1.m2 = getVoxelIdx(v.indices + BasisIdx{0,1,1});

                f2.m0 = getVoxelIdx(v.indices + BasisIdx{1,0,1});
                f2.m1 = getVoxelIdx(v.indices + BasisIdx{1,1,1});
                f2.m2 = getVoxelIdx(v.indices + BasisIdx{0,1,1});

                masses[f1.m0].isOnBoundary = true;
                masses[f1.m1].isOnBoundary = true;
                masses[f1.m2].isOnBoundary = true;
                masses[f2.m0].isOnBoundary = true;
                masses[f2.m1].isOnBoundary = true;
                masses[f2.m2].isOnBoundary = true;

                faces.push_back(f1);
                faces.push_back(f2);
            }
        }
    }

    // Gets number of non-air adjacent voxels
    uint16_t VoxelRobot::ConnectedVoxelRecurse(BasisIdx ind, std::vector<bool>& visit_list) {
        uint16_t numConnected = 0;
        uint16_t idx = getVoxelIdx(ind);

        if(!isValidIdx(ind)) return 0;
        if(visit_list[idx] == true) return 0;

        visit_list[idx] = true;
        Voxel v = voxels[idx];
        
        if(v.mat == materials::air) {
            return 0;
        }

        numConnected = 1;

        // 100, 010, 001
        int x = 1, y = 0, z = 0;
        // voxel postive and negative cardinal directions
        Voxel v_pos, v_neg;
        int tempx, tempy;
        for(uint8_t k = 0; k < 3; k++) {
            BasisIdx pos_offset = {x,y,z};
            numConnected += ConnectedVoxelRecurse(ind + pos_offset, visit_list);

            BasisIdx neg_offset = {-x,-y,-z};
            numConnected += ConnectedVoxelRecurse(ind + neg_offset, visit_list);

            tempx = x;
            tempy = y;
            x = z;
            y = tempx;
            z = tempy;
        }

        return numConnected;
    }

    void VoxelRobot::CopyFromVoxelRecurse(BasisIdx ind, VoxelRobot& R, std::vector<bool>& visit_list) {
        if(!isValidIdx(ind)) return;
        uint16_t idx = getVoxelIdx(ind);

        if(visit_list[idx] == true) return;

        visit_list[idx] = true;

        Voxel v = voxels[idx];

        if(v.mat == materials::air) return;

        R.voxels[idx] = voxels[idx];

        // voxel postive and negative cardinal directions
        Voxel v_pos, v_neg;
        int x = 1, y = 0, z = 0;
        int tempx, tempy;
        for(uint8_t k = 0; k < 3; k++) {
            BasisIdx pos_offset = {x,y,z};
            CopyFromVoxelRecurse(ind + pos_offset, R, visit_list);

            BasisIdx neg_offset = {-x,-y,-z};
            CopyFromVoxelRecurse(ind + neg_offset, R, visit_list);

            tempx = x;
            tempy = y;
            x = z;
            y = tempx;
            z = tempy;
        }
    }

    void VoxelRobot::Strip() {
        std::vector<bool> visited(voxels.size(), false);
        std::vector<uint> connected_count(voxels.size());
        uint16_t idx = 0;
        uint16_t maxIdx = 0;
        for(Voxel& v : voxels) {
            connected_count[idx] = ConnectedVoxelRecurse(v.indices, visited);
            if(connected_count[idx] > connected_count[maxIdx]) maxIdx = idx;
            idx++;
        }
        VoxelRobot R = *this;
        for(Voxel& v: R.voxels) v.mat = materials::air;
        
        visited = std::vector<bool>(voxels.size(), false);
        CopyFromVoxelRecurse(getVoxelBasisIdx(maxIdx), R, visited);
        *this = R;
    }

    void RunBatchBuild(std::vector<VoxelRobot>& robots, size_t begin, size_t end) {
        for(size_t i = begin; i < end; i++) {
            robots[i].Build();
        }
    }

    void VoxelRobot::BatchBuild(std::vector<VoxelRobot>& robots) {
        const auto processor_count = std::thread::hardware_concurrency();
        unsigned int active_threads = min(robots.size(), processor_count);
        unsigned int robots_per_thread = robots.size() / active_threads;
        
        std::vector<std::thread> threads;
        size_t begin, end;
        for(unsigned int i = 0; i < active_threads; i++) {
            begin = i*robots_per_thread;
            end = min((i+1)*robots_per_thread, robots.size());
            std::thread t(RunBatchBuild, std::ref(robots), begin, end);
            threads.push_back(std::move(t));
        }

        for(size_t i = 0; i < active_threads; i++) {
            threads[i].join();
        }
    }

    void VoxelRobot::Build() {
        Clear();
        Strip();

        mVolume = 0;

        std::vector<bool> visited(voxels.size());
        for(uint16_t i = 0; i < voxels.size(); i++) {
            if(voxels[i].mat != materials::air) mVolume++;
            Mass m(i, voxels[i].base, voxels[i].mat);
            addMass(m);
            visited[i] = false;
        }
        
        std::vector<Spring> _springs;
        BuildSpringsRecurse(_springs, {0,0,0}, visited);
        setSprings(_springs);
        BuildFaces();

        ShiftX();
        ShiftY();
        
        updateBaseline();
    }

    void VoxelRobot::BuildFromCircles() {
        for(Voxel& v : voxels) {
            v.mat = materials::air;
            if((uint16_t) v.indices.x == xCount-1 || (uint16_t) v.indices.y == yCount-1 || (uint16_t) v.indices.z == zCount-1) {
                continue;
            }
            std::vector<Material> mats;
            uint32_t matEncoding = 0x00u;
            float dist;
            for(Circle& c : circles) {
                dist = (v.center-c.center).norm();
                if(dist < c.radius) {
                    matEncoding |= c.mat.encoding;
                }
            }
            v.mat = materials::decode(matEncoding);
        }

        Build();
    }

    void VoxelRobot::Initialize() {
        circles.push_back({materials::bone});
        circles.push_back({materials::bone});
        circles.push_back({materials::tissue});
        circles.push_back({materials::tissue});
        circles.push_back({materials::adductor_muscle0});
        circles.push_back({materials::adductor_muscle0});
        // circles.push_back({materials::abductor_muscle0});
        // circles.push_back({materials::abductor_muscle0});

        xCount = resolution*xSize;
        yCount = resolution*ySize;
        zCount = resolution*zSize;
        uint16_t numVoxels = xCount*yCount*zCount;
        voxels = std::vector<Voxel>(numVoxels);

        Eigen::Vector3f center_correction = (1.0f/resolution)*Eigen::Vector3f(0.5f,0.5f,0.5f);
        for(uint16_t i = 0; i < voxels.size(); i++) {
            BasisIdx indices = getVoxelBasisIdx(i);
            Eigen::Vector3f base((1.0f/resolution)*(indices.x),
                        (1.0f/resolution)*(indices.y),
                        (1.0f/resolution)*(indices.z));
            Eigen::Vector3f center = base + center_correction;
            if((uint16_t) indices.x == xCount-1 || (uint16_t) indices.y == yCount-1 || (uint16_t) indices.z == zCount-1)
                voxels[i] = {i, indices, center, base, materials::air};
            else
                voxels[i] = {i, indices, center, base, materials::bone};
        }
        // Build();
    }

    VoxelRobot::VoxelRobot()
    {
        Initialize();
    }

    void VoxelRobot::Mutate() {
        switch(VoxelRobot::repr) {
        case ENCODE_RADIUS:
        {
            uint16_t idx = rand() % circles.size();
            circles[idx].Randomize(xSize,ySize,zSize);
        }
        break;

        case ENCODE_DIRECT:
        case ENCODE_FUNC:
        case ENCODE_SHAPE_ACT:
        default:
        {
            uint16_t idx = rand() % voxels.size();
            voxels[idx].setRandomMaterial();

            Build();
        }
        }
    }

    void VoxelRobot::Randomize() {
        for(Voxel& v : voxels) {
            v.mat = materials::air;
        }
        
        switch(VoxelRobot::repr) {
        case ENCODE_RADIUS:
            for(Circle& c : circles)
                c.Randomize(xSize,ySize,zSize);
            BuildFromCircles();
            break;
        case ENCODE_DIRECT: 
        default:
            for(Voxel& v : voxels) {
                if((uint16_t) v.indices.x < xCount-1 && (uint16_t) v.indices.y < yCount-1 && (uint16_t) v.indices.z < zCount-1)
                    v.setRandomMaterial();
            }
            Build();
        }
        mAge = 0;
    }

    CandidatePair<VoxelRobot> VoxelRobot::TwoPointChildren(const CandidatePair<VoxelRobot>& parents) {
        CandidatePair<VoxelRobot> children = {parents.first, parents.second};

        uint16_t range = children.first.voxels.size();
        float uni = uniform(gen);
        // clear("%f\n",uni);
        uint sample_length = range*uni;
        uint point = (range-sample_length)*uniform(gen);

        for(uint16_t i = point; i < sample_length; i++) {
            swap(children.first.voxels[i], children.second.voxels[i]);
        }

        children.first.Build();
        children.second.Build();

        return children;
    }

    CandidatePair<VoxelRobot> VoxelRobot::RadiusChildren(const CandidatePair<VoxelRobot>& parents) {
        CandidatePair<VoxelRobot> children = {parents.first, parents.second};

        uint num_swapped = rand() % children.first.circles.size();

        uint c1, c2;
        for(uint i = 0; i < num_swapped; i++) {
            c1 = rand() % num_swapped;
            do {
                c2 = rand() % num_swapped;
            } while (c1 != c2);
            swap(children.first.circles[c1],children.second.circles[c2]);
        }

        children.first.BuildFromCircles();
        children.second.BuildFromCircles();

        return children;
    }

    CandidatePair<VoxelRobot> VoxelRobot::Crossover(const CandidatePair<VoxelRobot>& parents) {
        CandidatePair<VoxelRobot> children;
        switch(repr) {
            case ENCODE_DIRECT: 
                children = TwoPointChildren(parents);
                break;
            case ENCODE_RADIUS:
                children = RadiusChildren(parents);
                break;
            default:
                children = TwoPointChildren(parents);
        }

        uint maxAge = parents.first.mAge;
        if(parents.second.mAge > maxAge)
            maxAge = parents.second.mAge;

        children.first.mAge = maxAge+1;
        children.second.mAge = maxAge+1;
        
        return children;
    }

    float VoxelRobot::Distance(const CandidatePair<VoxelRobot>& candidates) {
        float distance = 0;
        for(uint16_t i = 0; i < candidates.first.voxels.size(); i++) {
            distance += (candidates.first.voxels[i].mat.encoding != candidates.second.voxels[i].mat.encoding);
        }
        return distance;
    }

    std::vector<float> VoxelRobot::findDiversity(std::vector<VoxelRobot> pop) {
        size_t pop_size = pop.size();
        std::vector<float> diversity(pop_size, 0);
        // size_t v_size = pop[0].getVoxels().size();
        
        // TODO: Produces valgrind error
        // std::map<Material, float> mat_count;

        // TODO
        // for(size_t i = 0; i < v_size; i ++) {
        //     mat_count.clear();
        //     for(size_t j  = 0; j < pop_size; j++){
        //         Material mat = pop[j].getVoxels()[i].mat;
        //         if(mat_count.count(mat) == 0) mat_count[mat] = 0;
        //         // mat_count[mat]++;
        //     }
        // //     for(auto& count : mat_count) {
        // //         mat_count[count.first] = count.second / pop_size;
        // //     }
        // //     for(size_t j = 0; j < pop_size; j++) {
        // //         Voxel v = pop[j].getVoxels()[i];
        // //         diversity[j] += 1 - mat_count[v.mat];
        // //     }
        // }
        return diversity;
    }
    }
