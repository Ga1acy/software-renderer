#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include <vector>
#include <cmath>


const TGAColor white = TGAColor(255,255,255,255);
const TGAColor red = TGAColor(255,0,0,255);
const TGAColor blue = TGAColor(0,0,255,255);
const TGAColor green = TGAColor(0,255,0,255);
const int width = 200;
const int height = 200;
Model* model = nullptr;

inline Vec2i lerp_in_y(int y, Vec2i t0, Vec2i t1) {
    float alpha = (float)(y - t0.y) / (t1.y - t0.y);
    Vec2i t = t0 + (t1 - t0)*alpha;
    return t;
}


void line(Vec2i p0, Vec2i p1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {
        std::swap(p0.x,p0.y);
        std::swap(p1.x,p1.y);
        steep = true;
    }

    if (p0.x > p1.x) {
        std::swap(p0.x,p1.x);
        std::swap(p0.y,p1.y);
    }

    for (int x = p0.x; x <= p1.x; x++) {
//        //liner interpolation
//        //lerp(t,y0,y1) = t0 + x(t1-t0)
        float t = (x - p0.x) /(float)(p1.x - p0.x);
        int y = p0.y * (1.-t) + p1.y * t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage& image, TGAColor color) {
    if (t0.y > t1.y)
        std::swap(t0,t1);
    if (t0.y > t2.y)
        std::swap(t0,t2);
    if (t1.y > t2.y)
        std::swap(t1,t2);

    for (int y = t0.y; y <= t2.y; y++) {
        //phase 1
        if (y <= t1.y) {
            float x02 = lerp_in_y(y,t0,t2).x;
            float x01 = lerp_in_y(y, t0, t1).x;

            if (x02 > x01)
                std::swap(x02,x01);

            for (int x = x02; x <= x01; ++x) {
                image.set(x,y,color);
            }
        }
        //phase 2
        else if (y > t1.y){
            float x02 = lerp_in_y(y,t0,t2).x;
            float x12 = lerp_in_y(y, t1, t2).x;

            if (x02 > x12)
                std::swap(x02,x12);

            for (int x = x02; x <= x12; ++x) {
                image.set(x,y,color);
            }
        }
    }
}

int main(int argc, char** argv) {
    TGAImage image(width,height,TGAImage::RGB);
    Vec2i t0[3] = {Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80)};
    Vec2i t1[3] = {Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180)};
    Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};
    triangle(t0[0], t0[1], t0[2], image, red);
    triangle(t1[0], t1[1], t1[2], image, white);
    triangle(t2[0], t2[1], t2[2], image, green);

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}


