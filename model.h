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

struct Face {
    std::vector<int> vertexIndices;
    std::vector<int> texcoordsIndices;
};

class Model {
private:
    std::vector<Vec3f> verts_;
    std::vector<Face> faces_;
    std::vector<Vec2f> texcoords_;

public:
    Model(const char *filename);
    ~Model();
    int nverts();
    int nfaces();
    Vec3f vert(int i);
    Face face(int idx);
    Vec2f& getTexCoord(int index);
};

#endif //__MODEL_H__