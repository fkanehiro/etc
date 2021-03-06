#include <iostream>
#include <fstream>
#include <hrpModel/Link.h>
#include "CdShapeUtil.h"

void loadCdShapes(hrp::BodyPtr i_body, const char *i_filename,
                  std::vector<CdShape> &o_shapes)
{
    std::ifstream ifs(i_filename);
    if (!ifs.is_open()) {
        std::cerr << "failed to open(" << i_filename << ")" << std::endl;
        return;
    }
    std::string name;
    int type;
    hrp::Vector3 p1, p2;
    double r;
    
    ifs >> name;
    while(!ifs.eof()){
        ifs >> type;
        hrp::Link *l = i_body->link(name);
        if (type == CdShape::SPHERE){
            for (int i=0; i<3; i++) ifs >> p1[i];
            ifs >> r;
            if (name[0] != '#') o_shapes.push_back(CdShape(l, p1, r));
            }else{
            for (int i=0; i<3; i++) ifs >> p1[i];
            for (int i=0; i<3; i++) ifs >> p2[i];
            ifs >> r;
            if (name[0] != '#') o_shapes.push_back(CdShape(l, p1, p2, r));
        }
        ifs >> name;
    }
}

void loadCdPairs(const std::vector<CdShape> &i_shapes,
                 const char *i_filename, 
                 std::vector<std::pair<const CdShape *, const CdShape *> > &o_pairs)
{
    std::ifstream ifs(i_filename);
    if (!ifs.is_open()) {
        std::cerr << "failed to open(" << i_filename << ")" << std::endl;
        return;
    }
    std::string name1, name2;
    ifs >> name1;
    while (!ifs.eof()){
        ifs >> name2;
        if (name1[0] == '#') continue;
            for (unsigned int i=0; i<i_shapes.size(); i++){
                const CdShape *s1 = &i_shapes[i];
                if (s1->link()->name == name1){
                    for (unsigned int j=0; j<i_shapes.size(); j++){
                        const CdShape *s2 = &i_shapes[j];
                        if (s2->link()->name == name2){
                            o_pairs.push_back(std::make_pair(s1,s2));
                        }
                    }
                }
            }
            ifs >> name1;
    }
    std::cout << o_pairs.size() << " pairs are defined" << std::endl;
}

bool checkCollision(const std::vector<std::pair<const CdShape *, const CdShape *> > &i_pairs, double i_tolerance)
{
    for (unsigned int i=0; i<i_pairs.size(); i++){
        const CdShape *s1 = i_pairs[i].first;
        const CdShape *s2 = i_pairs[i].second;
        if (s1->isColliding(s2, i_tolerance)){
            return true;
        }
    }
    return false;
}

bool checkCollision(const SphereTree &i_st, const std::vector<CdShape>& i_shapes, double i_tolerance)
{
    for (unsigned int i=0; i<i_shapes.size(); i++){
        if (i_shapes[i].type() == CdShape::SPHERE){
            if (i_st.isColliding(i_shapes[i].center(), 
                                 i_shapes[i].radius(),
                                 i_tolerance)){
                //std::cout << i_shapes[i].link()->name << " is colliding" << std::endl;
                return true;
            }
        }else{
            if (i_st.isColliding(i_shapes[i].center(0),
                                 i_shapes[i].center(1),
                                 i_shapes[i].radius(),
                                 i_tolerance)){
                //std::cout << i_shapes[i].link()->name << " is colliding" << std::endl;
                return true;
            }
        }
    }
    return false;
}

bool checkCollision(const SphereTree &i_st, const std::vector<CdShape>& i_shapes,
                    const std::vector<std::pair<const CdShape *, const CdShape *> > &i_pairs, double i_tolerance)
{
    if (checkCollision(i_st, i_shapes, i_tolerance)) return true;
    return checkCollision(i_pairs, i_tolerance);
}
