#include "element.h"
#include <vector>

namespace EvoDevo {

    /*
    * 1) Sorts masses such that boundary masses are continguous in memory 
    * 2) Updates mesh element mass id's to match new mass id's
    */
    void Element::SortBoundaryMasses() {
        std::vector<uint16_t> idxMap(masses.size());

        // Can push to batched GPU sort if need be
        std::sort(masses.begin(), masses.end(),[](const Mass& a, const Mass& b) {
                return a.isOnBoundary > b.isOnBoundary;
            });
        for(uint16_t i = 0; i < masses.size(); i++) {
            idxMap[masses[i].id] = i;
            masses[i].id = i;
            if(masses[i].isOnBoundary) boundaryCount++;
        }
        for(auto& s : springs) {
            s.m0 = idxMap[s.m0];
            s.m1 = idxMap[s.m1];
        }
        for(auto& f : faces) {
            f.m0 = idxMap[f.m0];
            f.m1 = idxMap[f.m1];
            f.m2 = idxMap[f.m2];
        }
        for(auto& c : cells) {
            c.m0 = idxMap[c.m0];
            c.m1 = idxMap[c.m1];
            c.m2 = idxMap[c.m2];
            c.m3 = idxMap[c.m3];
        }
    }

    void Element::GroupFaces() {
        faceGroups.clear();
        
        std::vector<bool> massMarks(masses.size());
        std::vector<bool> faceMarks(faces.size(), false);
        uint16_t markCount = 0;

        Face f;
        std::vector<Face> group;
        while(markCount < faces.size()) {
            std::fill(massMarks.begin(), massMarks.end(), false);
            group.clear();
            faceGroups.push_back(std::vector<Face>());
            for(uint i = 0; i < faces.size(); i++) {
                if(faceMarks[i]) continue;
                faceMarks[i] = true;
                f = faces[i];
                group.push_back(f);
                massMarks[f.m0] = true;
                massMarks[f.m1] = true;
                massMarks[f.m2] = true;
                markCount += 1;
            }
            faceGroups.push_back(group);
        }
    }

    void Element::GroupSprings() {
        springGroups.clear();
        
        std::vector<bool> massMarks(masses.size());
        std::vector<bool> springMarks(faces.size(), false);
        uint16_t markCount = 0;

        Spring s;
        std::vector<Spring> group;
        while(markCount < springs.size()) {
            std::fill(massMarks.begin(), massMarks.end(), false);
            group.clear();
            springGroups.push_back(std::vector<Spring>());
            for(uint i = 0; i < springs.size(); i++) {
                if(springMarks[i]) continue;
                springMarks[i] = true;
                s = springs[i];
                group.push_back(s);
                massMarks[s.m0] = true;
                massMarks[s.m1] = true;
                markCount += 1;
            }
            springGroups.push_back(group);
        }
    }

    void Element::PreProcess() {
        if(preProcessed) return;
        SortBoundaryMasses();
        GroupFaces();
        GroupSprings();
        
        preProcessed = true;
    }

}