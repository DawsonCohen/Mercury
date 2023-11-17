#include <map>
#include <chrono>
#include <fstream>
#include <sstream>
#include "VoxelRobot.h"

VoxelRobot::Encoding VoxelRobot::repr = VoxelRobot::ENCODE_RADIUS;

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

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

void ShiftY(VoxelRobot& R) {
    bool setFlag = false;
    float minY = -1;
    for(const Mass& m : R.getMasses()) {
        if(!m.active) continue;
        if(!setFlag || m.protoPos.y() < minY) {
            setFlag = true;
            minY = m.protoPos.y();
        }
    }

    Eigen::Vector3f translation(0.0f,-minY,0.0f);
    R.translate(translation);
}

void ShiftX(VoxelRobot& R) {
    bool setFlag = false;
    float maxX = 0;
    for(const Mass& m : R.getMasses()) {
        if(!m.active) continue;
        if(!setFlag || m.protoPos.x() > maxX) {
            setFlag = true;
            maxX = m.protoPos.x();
        }
    }

    Eigen::Vector3f translation(-maxX,0.0f,0.0f);
    R.translate(translation);
}

// Note mMasses.size == voxels.size
// Builds spring from srcIdx to vIdx
void VoxelRobot::BuildSpringsRecurse(std::vector<Spring>& _springs, BasisIdx indices, std::vector<bool>& visit_list, uint srcIdx) {
    if(!isValidIdx(indices)) return;

    uint vIdx = getVoxelIdx(indices);

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
            for(uint i = 0; i < 4; i++) {
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
    // 1-10, 01-1,-101
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

// Gets number of non-air adjacent voxels
uint VoxelRobot::ConnectedVoxelRecurse(BasisIdx ind, std::vector<bool>& visit_list) {
    uint numConnected = 0;
    uint idx = getVoxelIdx(ind);

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
    for(uint k = 0; k < 3; k++) {
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

void VoxelRobot::Build() {
    Clear();
    Strip();

    mVolume = 0;

    std::vector<bool> visited(voxels.size());
    for(uint i = 0; i < voxels.size(); i++) {
        if(voxels[i].mat != materials::air) mVolume++;
        Mass m(i, voxels[i].base, voxels[i].mat);
        addMass(m);
        visited[i] = false;
    }
    
    std::vector<Spring> _springs;
    BuildSpringsRecurse(_springs, {0,0,0}, visited);
    setSprings(_springs);
    ShiftX(*this);
    ShiftY(*this);
    
    updateBaseline();
}

void VoxelRobot::BuildFromCircles() {
    for(Voxel& v : voxels) {
        if((uint) v.indices.x == xCount-1 || (uint) v.indices.y == yCount-1 || (uint) v.indices.z == zCount-1)
            continue;
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
    circles.push_back({materials::abductor_muscle0});
    circles.push_back({materials::abductor_muscle0});

    xCount = resolution*xSize;
    yCount = resolution*ySize;
    zCount = resolution*zSize;
    uint numVoxels = xCount*yCount*zCount;
    voxels = std::vector<Voxel>(numVoxels);

    Eigen::Vector3f center_correction = (1.0f/resolution)*Eigen::Vector3f(0.5f,0.5f,0.5f);
    for(uint i = 0; i < voxels.size(); i++) {
        BasisIdx indices = getVoxelBasisIdx(i);
        Eigen::Vector3f base((1.0f/resolution)*(indices.x),
                       (1.0f/resolution)*(indices.y),
                       (1.0f/resolution)*(indices.z));
        Eigen::Vector3f center = base + center_correction;
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
    mAge = 0;
}

CandidatePair<VoxelRobot> VoxelRobot::TwoPointChildren(const CandidatePair<VoxelRobot>& parents) {
    CandidatePair<VoxelRobot> children = {parents.first, parents.second};

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

void VoxelRobot::Duplicate(const VoxelRobot& R) {
    *this = R;
}

float VoxelRobot::Distance(const CandidatePair<VoxelRobot>& robots) {
    float dist = 0;
    std::vector<Voxel> s1 = robots.first.voxels;
    std::vector<Voxel> s2 = robots.second.voxels;
    for(size_t i = 0; i < s1.size(); i++) {
        dist += !(s1[i].mat == s2[i].mat);
    }
    return dist;
}

std::string VoxelRobot::Encode() const {
    std::string encoding;
    encoding += "type=VoxelRobot\n";
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

void VoxelRobot::Decode(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    
    uint ID;
    int xIdx, yIdx, zIdx;
    float   cx,cy,cz,
            bx, by, bz;
    Material mat;

    std::getline(file, line, '\n');
    std::getline(file, line, '\n');
    xSize = atof(line.data());
    std::getline(file, line, '\n');
    ySize = atof(line.data());
    std::getline(file, line, '\n');
    zSize = atof(line.data());
    std::getline(file, line, '\n');
    resolution = atof(line.data());
    
    while(file.peek() == 'v') {
        std::getline(file, line, 'v');
        std::getline(file, line, ',');
        ID = atoi(line.data());
        std::getline(file, line, ',');
        xIdx = atoi(line.data());
        std::getline(file, line, ',');
        yIdx = atoi(line.data());
        std::getline(file, line, ',');
        zIdx = atoi(line.data());
        std::getline(file, line, ',');
        cx = atof(line.data());
        std::getline(file, line, ',');
        cy = atof(line.data());
        std::getline(file, line, ',');
        cz = atof(line.data());
        std::getline(file, line, ',');
        bx = atof(line.data());
        std::getline(file, line, ',');
        by = atof(line.data());
        std::getline(file, line, ',');
        bz = atof(line.data());
        std::getline(file, line, ',');
        mat.k = atof(line.data());
        std::getline(file, line, ',');
        mat.dL0 = atof(line.data());
        std::getline(file, line, ',');
        mat.omega = atof(line.data());
        std::getline(file, line, ',');
        mat.phi = atof(line.data());
        std::getline(file, line, ',');
        mat.color.r = atof(line.data());
        std::getline(file, line, ',');
        mat.color.g = atof(line.data());
        std::getline(file, line, ',');
        mat.color.b = atof(line.data());
        std::getline(file, line, '\n');
        mat.color.a = atof(line.data());


        Eigen::Vector3f center(cx,cy,cz);
        Eigen::Vector3f base(bx,by,bz);

        Voxel v{ID, {xIdx,yIdx,zIdx}, center, base, mat};
        voxels.push_back(v);
    }
}