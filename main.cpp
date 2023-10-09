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

Vec3f computeBarycentric(const Vec3f* v, float x, float y) {
    float alpha = (x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*y + v[1].x*v[2].y - v[2].x*v[1].y) / (v[0].x*(v[1].y- v[2].y) + (v[2].x - v[1].x)*v[0].y + v[1].x*v[2].y - v[2].x*v[1].y);
    float beta = (x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*y + v[2].x*v[0].y - v[0].x*v[2].y) / (v[1].x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*v[1].y + v[2].x*v[0].y - v[0].x*v[2].y);
    float gamma = (x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*y + v[0].x*v[1].y - v[1].x*v[0].y) / (v[2].x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*v[2].y + v[0].x*v[1].y - v[1].x*v[0].y);


    return {alpha,beta,gamma};
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

void triangle(Vec3f* vertices , float* zBuffer, TGAImage& image, TGAColor color) {
    Vec2f bounding_box_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bouding_box_max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    //calculate the bouding box
    for (int i = 0; i < 3; i++) {
        bounding_box_min.x = std::max(0.f, std::min(vertices[i].x , bounding_box_min.x));
        bounding_box_min.y = std::max(0.f,std::min(vertices[i].y, bounding_box_min.y));
        bouding_box_max.x = std::min(clamp.x, std::max(bouding_box_max.x,vertices[i].x));
        bouding_box_max.y = std::min(clamp.y, std::max(bouding_box_max.y,vertices[i].y));
    }

    Vec3f P;
    for (P.x = bounding_box_min.x; P.x <= bouding_box_max.x; P.x++) {
        for (P.y = bounding_box_min.y; P.y <= bouding_box_max.y; P.y++) {
            //compute barycentric coordinates to confirm if (i,j) inside triangle
            Vec3f P_bc = computeBarycentric(vertices,P.x,P.y);
            if (P_bc.x <0 || P_bc.y < 0 || P_bc.z < 0)
                continue;
            P.z = 0;
            //if P inside the triangle, we need to interpolate to z value
            //P_bc.x stands for alpha, .y for beta and .z for gamma
            //thus the formula is P.z = alpha * A.z + beta * B.z + gamma * C.z
            P.z = P_bc.x * vertices[0].z + P_bc.y * vertices[1].z + P_bc.z * vertices[2].z;
            if (zBuffer[int(P.x + P.y * width)] < P.z) {
                zBuffer[int(P.x + P.y * width)]  = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main(int argc, char** argv) {

    TGAImage image(width, height, TGAImage::RGB);
    model = new Model("../obj/african_head.obj");

    float* zBuffer = new float[width * height];
    //initialize zBuffer, when i < 0, automatically jump out for loop
    for (int i = width * height; i--; zBuffer[i] = - std::numeric_limits<float>::max());
    Vec3f light_dir(0,0,-1);

    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f screen_coords[3], world_coords[3];
        for (int j = 0; j < 3; j++) {
            //get one of three vertex in a face
            world_coords[j] = model->vert(face[j]);
            screen_coords[j] = world2screen(world_coords[j]);
        }
        //the normal of a face
        Vec3f n = cross((world_coords[2] - world_coords[0]),(world_coords[1] - world_coords[0]));
        n.normalize();
        float intensity = n * light_dir;

        if (intensity > 0)
            triangle(screen_coords,zBuffer,image,TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));

    }
    image.flip_vertically(); // to place the origin in the bottom left corner of the image
    image.write_tga_file("flat-shading-with-zBuffer.tga");
    delete model;
    return 0;
}


