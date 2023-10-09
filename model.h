//
// Created by galacy on 2023/10/6.
//

#ifndef TINYRENDERER_MODEL_H
#define TINYRENDERER_MODEL_H

#endif //TINYRENDERER_MODEL_H
#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"

class Model {
private:
    std::vector<Vec3f> verts_;
    std::vector<std::vector<int> > faces_;
public:
    Model(const char *filename);
    ~Model();
    int nverts();
    int nfaces();
    Vec3f vert(int i);
    std::vector<int> face(int idx);
};

#endif //__MODEL_H__