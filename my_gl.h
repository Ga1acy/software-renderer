//
// Created by galacy on 2023/10/14.
//

#ifndef TINYRENDERER_MY_GL_H
#define TINYRENDERER_MY_GL_H



#include "tgaimage.h"
#include "geometry.h"

const float depth = 2000.f;
extern Matrix ModelView;
extern Matrix Viewport;
extern Matrix Projection;

void viewport(int x, int y, int w, int h);
void projection(float coeff); // coeff = -1/c
void lookat(Vec3f eye, Vec3f center, Vec3f up);

struct IShader {
    virtual ~IShader();
    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};

void triangle(Vec4f *pts, IShader &shader, TGAImage &image, float* zbuffer);

#endif //TINYRENDERER_MY_GL_H

