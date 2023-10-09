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

Vec3f computeBarycentric(const Vec3f* v, float x, float y) {
    float alpha = (x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*y + v[1].x*v[2].y - v[2].x*v[1].y) / (v[0].x*(v[1].y- v[2].y) + (v[2].x - v[1].x)*v[0].y + v[1].x*v[2].y - v[2].x*v[1].y);
    float beta = (x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*y + v[2].x*v[0].y - v[0].x*v[2].y) / (v[1].x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*v[1].y + v[2].x*v[0].y - v[0].x*v[2].y);
    float gamma = (x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*y + v[0].x*v[1].y - v[1].x*v[0].y) / (v[2].x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*v[2].y + v[0].x*v[1].y - v[1].x*v[0].y);


    return {alpha,beta,gamma};
}

TGAColor getTextureColor(TGAImage& texture, float u, float v) {
    u = std::max(0.0f, std::min(1.0f, u));
    v = std::max(0.0f, std::min(1.0f, v));

    float x = u * texture.get_width();
    float y = v * texture.get_height();

    TGAColor color = texture.get(x,y);
    return TGAColor(color[2], color[1], color[0], 255);
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

//pts is the triangle's vertices, tex is triangle's vertices' correspond texture coordinates
void triangleWithTexture(Vec3f* pts, Vec2f* tex, float* zBuffer, TGAImage& image, TGAImage& texture) {
    Vec2f bounding_box_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bouding_box_max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    //calculate the bouding box
    for (int i = 0; i < 3; i++) {
        bounding_box_min.x = std::max(0.f, std::min(pts[i].x , bounding_box_min.x));
        bounding_box_min.y = std::max(0.f,std::min(pts[i].y, bounding_box_min.y));
        bouding_box_max.x = std::min(clamp.x, std::max(bouding_box_max.x,pts[i].x));
        bouding_box_max.y = std::min(clamp.y, std::max(bouding_box_max.y,pts[i].y));
    }

    Vec3f P;
    for (P.x = bounding_box_min.x; P.x <= bouding_box_max.x; P.x++) {
        for (P.y = bounding_box_min.y; P.y <= bouding_box_max.y; P.y++) {
            Vec3f bc = computeBarycentric(pts, P.x, P.y);
            if (bc.x < 0 || bc.y < 0 || bc.z < 0)
                continue;
            P.z = 0;
            //using interpolate to get pixel's z value and u-v coordinates in texture
            P.z = bc.x * pts[0].z + bc.y * pts[1].z + bc.z * pts[2].z;
            Vec2f uv = bc.x * tex[0] + bc.y * tex[1] + bc.z * tex[2];

            if (zBuffer[int(P.x + P.y * width)] < P.z) {
                zBuffer[int(P.x + P.y * width)]  = P.z;
                //v is from top to bottom, thus we use 1-uv.y to flip
                image.set(P.x, P.y, getTextureColor(texture, uv.x, 1 - uv.y));
            }
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main(int argc, char** argv) {
    TGAImage image(width, height, TGAImage::RGB);
    TGAImage texture;
    model = new Model("../obj/african_head.obj");
    if (texture.read_tga_file("../obj/african_head_diffuse.tga")) {
        std::cout << "texture successfully loaded" << std::endl;
    } else {
        std::cerr << "Failed to load texture" << std::endl;
    }

    float* zBuffer = new float[width * height];
    //initialize zBuffer, when i < 0, automatically jump out for loop
    for (int i = width * height; i--; zBuffer[i] = - std::numeric_limits<float>::max());
    for (int i = 0; i < model->nfaces(); i++) {
        Face face = model->face(i);
        Vec3f screen_coords[3], world_coords[3];
        Vec2f tex_coords[3];
        for (int j = 0; j < 3; j++) {
            //get one of three vertex in a face
            world_coords[j] = model->vert(face.vertexIndices[j]);
            tex_coords[j] = model->getTexCoord(face.texcoordsIndices[j]);
            screen_coords[j] = world2screen(world_coords[j]);
        }
        triangleWithTexture(screen_coords,tex_coords,zBuffer,image,texture);
    }
    image.flip_vertically(); // to place the origin in the bottom left corner of the image
    image.write_tga_file("flat-shading-with-texture.tga");
    delete model;
//    delete[] zBuffer;
    return 0;
}


