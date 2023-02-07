#include <chrono>
#include <csignal>
#include <algorithm>
#include <glm/gtx/norm.hpp>
#include <map>
#include "VoxelRobot.h"

unsigned VoxelRobot::seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine VoxelRobot::gen = std::default_random_engine(VoxelRobot::seed);
std::uniform_real_distribution<> VoxelRobot::uniform = std::uniform_real_distribution<>(0.0,1.0);

VoxelRobot::Encoding repr = VoxelRobot::ENCODE_DIRECT;

float VoxelRobot::Distance() {
    glm::vec3 mean_pos = glm::vec3(0.0f);
    float i = 0;
    for(Mass& m : masses) {
        mean_pos = mean_pos + (m.pos - mean_pos) * 1.0f/(i+1);
        i++;
    }
    return glm::l2Norm(mean_pos);
}

void Circle::Randomize(float xlim, float ylim, float zlim) {
    static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    static std::default_random_engine gen = std::default_random_engine(seed);
    static std::uniform_real_distribution<> uniform = std::uniform_real_distribution<>(0.0,1.0);
    
    glm::vec3 new_center;
    new_center.x = xlim*uniform(gen);
    new_center.y = ylim*uniform(gen);
    new_center.z = zlim*uniform(gen);
    center = new_center;
    radius = max_radius * uniform(gen);
}

void ShiftY(VoxelRobot& R) {
    bool setFlag = false;
    float minY = -1;
    for(const Mass& m : R.getMasses()) {
        if(!m.active) continue;
        if(!setFlag || m.protoPos.y < minY) {
            setFlag = true;
            minY = m.protoPos.y;
        }
    }

    R.translate(glm::vec3(0.0f,-minY,0.0f));
}

void ShiftX(VoxelRobot& R) {
    bool setFlag = false;
    float maxX = 0;
    for(const Mass& m : R.getMasses()) {
        if(!m.active) continue;
        if(!setFlag || m.protoPos.x > maxX) {
            setFlag = true;
            maxX = m.protoPos.x;
        }
    }

    R.translate(glm::vec3(-maxX,0.0f,0.0f));
}

// Note mMasses.size == voxels.size
void VoxelRobot::BuildSpringsRecurse(std::vector<Spring>& _springs, BasisIdx indices, bool* visit_list, uint srcIdx) {
    if(!isValidInd(indices)) return;

    uint vIdx = getVoxelIdx(indices);

    Voxel v = voxels[vIdx];
    Voxel v_src = voxels[srcIdx];

    if(srcIdx != vIdx) {
        // Spring mean length
        float L = glm::l2Norm(v_src.base - v.base);
        
        //  Get material of all owning voxels
        BasisIdx path = indices - v_src.indices;
        Material mat;

        BasisIdx neighbors[4];
        for(uint i = 0; i < 4; i++) {
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
            std::vector<Material> nMats;
            for(uint i = 0; i < 4; i++) {
                if(!isValidInd(neighbors[i])) continue;
                Voxel neighbor = voxels[getVoxelIdx(neighbors[i])];
                nMats.push_back(neighbor.mat);
            }
            mat = Material::avg(nMats);
        } else if(abs(path.x)+abs(path.y)+abs(path.z) == 2) { // diagonal
            BasisIdx nprox = {!path.x,!path.y,!path.z};
            BasisIdx neighborIdx = v_src.indices - nprox;
            if(path.x >= 0 && path.y >= 0 && path.z >= 0 ){
                if(!isValidInd(neighborIdx)) {
                    mat = v_src.mat;
                } else {
                    std::vector<Material> nMats;
                    Voxel neighbor = voxels[getVoxelIdx(neighborIdx)];
                    nMats.push_back(neighbor.mat);
                    nMats.push_back(v_src.mat);
                    mat = Material::avg(nMats);
                }
            } else {
                BasisIdx srcprx = {path.x==-1,path.y==-1,path.z==-1};
                BasisIdx matSrcIdx = v_src.indices - srcprx;
                Voxel v_matsrc = voxels[getVoxelIdx(matSrcIdx)];
                BasisIdx neighborIdx = matSrcIdx - nprox;

                if(!isValidInd(neighborIdx)) {
                    mat = v_matsrc.mat;
                } else {
                    std::vector<Material> nMats;
                    Voxel neighbor = voxels[getVoxelIdx(neighborIdx)];
                    nMats.push_back(neighbor.mat);
                    nMats.push_back(v_matsrc.mat);
                    mat = Material::avg(nMats);
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

        Spring s{srcIdx,vIdx,L,L,mat};
        _springs.push_back(s);
    }


    if(visit_list[vIdx]) return;
    visit_list[vIdx] = true;
    //  Adds the following to the basis indices:
    //  001, 010, 011, 100, 101, 110, 111
    int x,y,z;
    uint tempx,tempy;
    for(uint k = 1; k < 8; k++) {
        x = k%2;
        y = (k/2)%2;
        z = (k/4)%2;
        BasisIdx offset = {x,y,z};
        BuildSpringsRecurse(_springs, indices + offset, visit_list, vIdx);
    }

    //  Adds the following to the basis indices:
    // 1-10, 0-11,-101
    x = 1; y = -1; z = 0;
    for(uint k = 0; k < 3; k++) {
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
    for(uint k = 0; k < 3; k++) {
        BasisIdx offset = {x,y,z};
        BuildSpringsRecurse(_springs, indices + offset, visit_list, vIdx);
        tempx = x;
        tempy = y;
        x = z;
        y = tempx;
        z = tempy;
    }
}

uint VoxelRobot::StripRecurse(BasisIdx ind, std::vector<bool>& visit_list) {
    uint numConnected = 0;
    uint idx = getVoxelIdx(ind);

    if(!isValidInd(ind)) return 0;
    if(visit_list[idx] == true) return 0;

    visit_list[idx] = true;
    Voxel v = voxels[idx];
    
    if(v.mat == materials::air) {
        return 0;
    }

    numConnected = 1;

    // 100, 010,001
    int x = 1, y = 0, z = 0;
    // voxel postive and negative cardinal directions
    Voxel v_pos, v_neg;
    int tempx, tempy;
    for(uint k = 0; k < 3; k++) {
        BasisIdx pos_offset = {x,y,z};
        numConnected += StripRecurse(ind + pos_offset, visit_list);

        BasisIdx neg_offset = {-x,-y,-z};
        numConnected += StripRecurse(ind + neg_offset, visit_list);

        tempx = x;
        tempy = y;
        x = z;
        y = tempx;
        z = tempy;
    }

    return numConnected;
}

void VoxelRobot::CopyFromVoxelRecurse(BasisIdx ind, VoxelRobot& R, std::vector<bool>& visit_list) {
    if(!isValidInd(ind)) return;
    uint idx = getVoxelIdx(ind);

    if(visit_list[idx] == true) return;

    visit_list[idx] = true;

    Voxel v = voxels[idx];

    if(v.mat == materials::air) return;

    R.voxels[idx] = voxels[idx];

    // voxel postive and negative cardinal directions
    Voxel v_pos, v_neg;
    int x = 1, y = 0, z = 0;
    int tempx, tempy;
    for(uint k = 0; k < 3; k++) {
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
    uint idx = 0;
    uint maxIdx = 0;
    for(Voxel& v : voxels) {
        connected_count[idx] = StripRecurse(v.indices, visited);
        if(connected_count[idx] > connected_count[maxIdx]) maxIdx = idx;
        idx++;
    }
    VoxelRobot R = *this;
    for(Voxel& v: R.voxels) v.mat = materials::air;

    visited = std::vector<bool>(voxels.size(), false);
    CopyFromVoxelRecurse(getVoxelBasisIdx(maxIdx), R, visited);
    *this = R;
}

void VoxelRobot::Build() {
    Clear();
    Strip();
    // Mass::nextIndex = 0;

    mVolume = 0;

    bool visited[voxels.size()];
    for(uint i = 0; i < voxels.size(); i++) {
        if(voxels[i].mat != materials::air) mVolume++;
        addMass(Mass{i,voxels[i].base,voxels[i].mat.color});
        visited[i] = false;
    }
    
    std::vector<Spring> _springs;
    BuildSpringsRecurse(_springs, {0,0,0}, visited);
    setSprings(_springs);
    ShiftX(*this);
    ShiftY(*this);

    mCOM = calcMeanPos(*this);
    mSkew = calcSkew(*this);
}

void VoxelRobot::BuildFromCircles() {
    for(Voxel& v : voxels) {
        std::vector<Material> mats;
        mats.push_back(materials::air);
        float dist;
        for(Circle& c : circles) {
            dist = glm::l2Norm(v.center-c.center);
            if(dist < c.radius) mats.push_back(materials::bone);
        }
        v.mat = Material::avg(mats);
    }

    Build();
}

void VoxelRobot::Initialize() {
    circles.push_back({BONE});
    circles.push_back({BONE});
    circles.push_back({TISSUE});
    circles.push_back({TISSUE});
    circles.push_back({AGONIST_MUSCLE});
    circles.push_back({AGONIST_MUSCLE});
    circles.push_back({ANTAGOINST_MUSCLE});
    circles.push_back({ANTAGOINST_MUSCLE});

    xCount = resolution*xSize;
    yCount = resolution*ySize;
    zCount = resolution*zSize;
    uint numVoxels = xCount*yCount*zCount;
    voxels = std::vector<Voxel>(numVoxels);

    glm::vec3 center_correction = (1.0f/resolution)*glm::vec3(-0.5f,0.5f,-0.5f);
    for(uint i = 0; i < voxels.size(); i++) {
        BasisIdx indices = getVoxelBasisIdx(i);
        glm::vec3 base((1.0f/resolution)*(indices.x),
                       (1.0f/resolution)*(indices.y),
                       (1.0f/resolution)*(indices.z));
        glm::vec3 center = base + center_correction;
        if((uint) indices.x == xCount-1 || (uint) indices.y == yCount-1 || (uint) indices.z == zCount-1)
            voxels[i] = {i, indices, center, base, materials::air};
        else
            voxels[i] = {i, indices, center, base, materials::bone};
    }
    Build();
}

VoxelRobot::VoxelRobot()
{
    Initialize();
}

void VoxelRobot::Mutate() {
    switch(VoxelRobot::repr) {
    case ENCODE_RADIUS:
    {
        int idx = rand() % circles.size();
        circles[idx].Randomize(xSize,ySize,zSize);
    }
    break;

    case ENCODE_DIRECT:
    case ENCODE_FUNC:
    case ENCODE_SHAPE_ACT:
    default:
    {
        int idx = rand() % voxels.size();
        voxels[idx].setRandomMaterial();

        Build();
    }
    }
}

void VoxelRobot::Randomize() {
    switch(VoxelRobot::repr) {
    case ENCODE_RADIUS:
        for(Circle& c : circles)
            c.Randomize(xSize,ySize,zSize);
        BuildFromCircles();
        break;
    case ENCODE_DIRECT: 
    default:
        for(Voxel& v : voxels) {
            if((uint) v.indices.x < xCount-1 && (uint) v.indices.y < yCount-1 && (uint) v.indices.z < zCount-1)
                v.setRandomMaterial();
        }
        Build();
    }
}

VoxelRobotPair VoxelRobot::TwoPointChildren(const VoxelRobotPair& parents) {
    VoxelRobotPair children = {parents.first, parents.second};

    uint range = children.first.voxels.size();
    float uni = uniform(gen);
    // clear("%f\n",uni);
    uint sample_length = range*uni;
    uint point = (range-sample_length)*uniform(gen);

    for(uint i = point; i < sample_length; i++) {
        swap(children.first.voxels[i], children.second.voxels[i]);
    }

    children.first.Build();
    children.second.Build();

    return children;
}

VoxelRobotPair VoxelRobot::RadiusChildren(const VoxelRobotPair& parents) {
    VoxelRobotPair children = {parents.first, parents.second};

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

VoxelRobotPair VoxelRobot::Crossover(const VoxelRobotPair& parents) {
    VoxelRobotPair children;
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

    int maxAge = parents.first.mAge;
    if(parents.second.mAge > maxAge)
        maxAge = parents.second.mAge;

    children.first.mAge = maxAge+1;
    children.second.mAge = maxAge+1;
    
    return children;
}

void VoxelRobot::Duplicate(const VoxelRobot& R) {
    *this = R;
}

glm::vec3 VoxelRobot::calcMeanPos(VoxelRobot& R) {
    glm::vec3 mean_pos = glm::vec3(0.0f);
    float i = 0;
    for(Voxel& v : R.voxels) {
        if(v.mat == materials::air) continue;
        mean_pos = mean_pos + (R.masses[v.ID].pos - mean_pos) * 1.0f/(i+1);
        i++;
    }

    return mean_pos;
}

glm::vec3 VoxelRobot::calcSkew(VoxelRobot& R) {
    glm::vec3 skew = glm::vec3(0.0f);
    float i = 0;
    for(Voxel& v : R.voxels) {
        if(v.mat == materials::air) continue;
        glm::vec3 dist = v.center - R.mCOM;
        glm::vec3 loc_skew = glm::vec3(pow(dist.x,3), pow(dist.y,3), pow(dist.z,3));
        skew = skew + (loc_skew-skew) * 1.0f/(i+1);
        i++;
    }
    skew.y = 0;

    return skew;
}

void VoxelRobot::calcFitness(VoxelRobot& R) {
    glm::vec3 mean_pos = calcMeanPos(R);

    // R.mFitness = glm::l2Norm(mean_pos);
    R.mFitness = mean_pos.x - R.mCOM.x;
}

float VoxelRobot::Distance(const VoxelRobotPair& robots) {
    float dist = 0;
    std::vector<Voxel> s1 = robots.first.voxels;
    std::vector<Voxel> s2 = robots.second.voxels;
    for(size_t i = 0; i < s1.size(); i++) {
        dist += !(s1[i].mat == s2[i].mat);
    }
    return dist;
}

std::string VoxelRobot::DirectEncode() const {
    std::string encoding;
    encoding += std::to_string(xSize) + "\n";
    encoding += std::to_string(ySize) + "\n";
    encoding += std::to_string(zSize) + "\n";
    encoding += std::to_string(resolution) + "\n";
    for(const Voxel& v : voxels) {
        encoding += "v " + v.to_string() + "\n";
    }
    return encoding;
}

std::vector<float> VoxelRobot::findDiversity(std::vector<VoxelRobot> pop) {
    size_t pop_size = pop.size();
    size_t v_size = pop[0].getVoxels().size();
    std::vector<float> diversity(pop_size, 0);
    
    std::map<Material, float> mat_count;

    // TODO
    for(size_t i = 0; i < v_size; i ++) {
        mat_count.clear();
        for(size_t j  = 0; j < pop_size; j++){
            Material mat = pop[j].getVoxels()[i].mat;
            if(mat_count.count(mat) == 0) mat_count[mat] = 0;
            mat_count[mat]++;
        }
        for(auto& count : mat_count) {
            mat_count[count.first] = count.second / pop_size;
        }
        for(size_t j = 0; j < pop_size; j++) {
            Voxel v = pop[j].getVoxels()[i];
            diversity[j] += 1 - mat_count[v.mat];
        }
    }
    return diversity;
}