#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include <vector>
#include <cmath>


const TGAColor white = TGAColor(255,255,255,255);
const TGAColor red = TGAColor(255,0,0,255);
const TGAColor blue = TGAColor(0,0,255,255);
const TGAColor green = TGAColor(0,255,0,255);
const int width = 800;
const int height = 800;
Model* model = nullptr;

inline Vec2i lerp_in_y(int y, Vec2i t0, Vec2i t1) {
    float alpha = (float)(y - t0.y) / (t1.y - t0.y);
    Vec2i t = t0 + (t1 - t0)*alpha;
    return t;
}

Vec3f computeBarycentric(const Vec2i* v, float x, float y) {
    float c1 = (x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*y + v[1].x*v[2].y - v[2].x*v[1].y) / (v[0].x*(v[1].y- v[2].y) + (v[2].x - v[1].x)*v[0].y + v[1].x*v[2].y - v[2].x*v[1].y);
    float c2 = (x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*y + v[2].x*v[0].y - v[0].x*v[2].y) / (v[1].x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*v[1].y + v[2].x*v[0].y - v[0].x*v[2].y);
    float c3 = (x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*y + v[0].x*v[1].y - v[1].x*v[0].y) / (v[2].x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*v[2].y + v[0].x*v[1].y - v[1].x*v[0].y);

    return {c1,c2,c3};
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

void triangle(Vec2i* vertices , TGAImage& image, TGAColor color) {
    Vec2i bounding_box_min(image.get_width() - 1, image.get_height() - 1);
    Vec2i bouding_box_max(0,0);
    Vec2i clamp(image.get_width() - 1, image.get_height() - 1);
    //calculate the bouding box
    for (int i = 0; i < 3; i++) {
        bounding_box_min.x = std::max(0,std::min(vertices[i].x, bounding_box_min.x));
        bounding_box_min.y = std::max(0,std::min(vertices[i].y, bounding_box_min.y));
        bouding_box_max.x = std::min(clamp.x, std::max(bouding_box_max.x,vertices[i].x));
        bouding_box_max.y = std::min(clamp.y, std::max(bouding_box_max.y,vertices[i].y));
    }

    for (int i = bounding_box_min.x; i <= bouding_box_max.x; i ++) {
        for (int j = bounding_box_min.y; j <= bouding_box_max.y; j++) {
            //compute barycentric coordinates to confirm if (i,j) inside triangle
            Vec3f P = computeBarycentric(vertices,i,j);
            if (P.x <0 || P.y < 0 || P.z < 0)
                continue;
            image.set(i,j,color);
        }
    }
}

int main(int argc, char** argv) {

    TGAImage image(width, height, TGAImage::RGB);
    model = new Model("../obj/african_head.obj");
    Vec2i* screen_coords;
    Vec3f world_coords[3];
    Vec3f light_dir(0,0,-1);

    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        for (int j = 0; j < 3; j++) {
            //get one of three vertex in a face
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = Vec2i((v.x + 1.) * width / 2, (v.y + 1.) * height / 2);
            world_coords[j] = v;
        }
        //the normal of a face
        Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;

        if (intensity > 0)
            triangle(screen_coords,image,TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));

    }
    image.flip_vertically(); // to place the origin in the bottom left corner of the image
    image.write_tga_file("flat-shading-with-light.tga");
    delete model;
    return 0;
}


