#include <vector>
#include <cmath>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const int width = 800;
const int height = 800;
const int depth = 255;

Model* model = nullptr;
int *zBuffer = nullptr;
Vec3f light_dir(0,0,-1);
Vec3f camera(0,0,3);

//Matrix to vector
Vec3f m2v(Matrix m) {
    return Vec3f(m[0][0] / m[3][0], m[1][0] / m[3][0], m[2][0] / m[3][0]);
}

//Vector to matrix
Matrix v2m(Vec3f v) {
    Matrix m(4,1);

    for (int i = 0; i < 3; i++)
        m[i][0] = v[i];

    m [3][0] = 1.f;
    return m;
}

Matrix viewport(int x, int y, int width, int height) {
    Matrix m = Matrix::identity(4);
    m[0][3] = (x + width) / 2.f;
    m[1][3] = (y + height) / 2.f;
    m[2][3] = depth / 2.f;

    m[0][0] = width / 2.f;
    m[1][1] = height / 2.f;
    m[2][2] = depth / 2.f;
    return m;
}

Vec3f computeBarycentric(const Vec3i* pts, Vec3i P) {
    Vec3f u =
            Vec3f(pts[2].x-pts[0].x, pts[1].x-pts[0].x, pts[0].x-P.x)^
            Vec3f(pts[2].y-pts[0].y, pts[1].y-pts[0].y, pts[0].y-P.y)
    ;
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
}

void triangle(Vec3i t0, Vec3i t1, Vec3i t2,
              Vec2i uv0, Vec2i uv1, Vec2i uv2,
              TGAImage& image, float intensity, int* zBuffer) {
    //compute bouding box
    Vec3i bboxmin(image.get_width() - 1, image.get_height() - 1, 0);
    Vec3i bboxmax(0, 0, 0);
    Vec3i clamp (image.get_width() - 1, image.get_height() - 1, 0);
    bboxmin.x = std::max(0, std::min(bboxmin.x, std::min(t0.x, std::min(t1.x, t2.x))));
    bboxmin.y = std::max(0, std::min(bboxmin.y, std::min(t0.y, std::min(t1.y, t2.y))));
    bboxmax.x = std::min(clamp.x, std::max(t0.x, std::max(t1.x, t2.x)));
    bboxmax.y = std::min(clamp.y, std::max(t0.y, std::max(t1.y, t2.y)));

    Vec3i pts[3] = {t0, t1, t2};
    Vec3i P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f bc = computeBarycentric(pts, P);
            //check if P inside triangle
            if (bc.x < 0 || bc.y < 0 || bc.z < 0)
                continue;
            P.z = bc.x * t0.z + bc.y * t1.z + bc.z * t2.z;
            Vec2i uvP;
            uvP.x = bc.x * uv0.x + bc.y * uv1.x + bc.z * uv2.x;
            uvP.y = bc.x * uv0.y + bc.y * uv1.y + bc.z * uv2.y;
            int idx = P.x + P.y * image.get_width();
            if (zBuffer[idx] < P.z) {
                zBuffer[idx] = P.z;
                TGAColor color = model->diffuse(uvP);
                image.set(P.x, P.y, TGAColor(color.r * intensity, color.g * intensity, color.b * intensity));
            }
        }
    }
}



int main(int argc, char** argv) {

    model = new Model("../obj/african_head.obj");

    zBuffer = new int[width * height];

    for (int i=0; i<width*height; i++) {
        zBuffer[i] = std::numeric_limits<int>::min();
    }

    {
        TGAImage image(width, height, TGAImage::RGB);
        Matrix Projection = Matrix::identity(4);
        Matrix ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
        Projection[3][2] = -1.f / camera.z;
        for (int i = 0; i < model->nfaces(); i++) {
            std::vector<int> face = model->face(i);
            Vec3i screen_coords[3];
            Vec3f world_coords[3];
            for (int j = 0; j < 3; j++) {
                //get one of three vertex in a face
                Vec3f v = model->vert(face[j]);
                screen_coords[j] = m2v(ViewPort * Projection * v2m(v));
                world_coords[j] = v;
            }
            //the normal of a face
            Vec3f n = ((world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0])).normalize();
            float intensity = n * light_dir;

            if (intensity > 0) {
                Vec2i uv[3];
                for (int k = 0; k < 3; k++) {
                    uv[k] = model->uv(i, k);
                }
                triangle(screen_coords[0], screen_coords[1], screen_coords[2],
                         uv[0], uv[1], uv[2],
                         image, intensity, zBuffer);
            }
        }
        image.flip_vertically(); // to place the origin in the bottom left corner of the image
        image.write_tga_file("output.tga");
    }


    {
        TGAImage zbImage(width,height,TGAImage::GRAYSCALE);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++){
                zbImage.set(i, j, TGAColor(zBuffer[i + j * width],1));
            }
        }
        zbImage.flip_vertically();
        zbImage.write_tga_file("zbuffer.tga");
    }
    delete model;
    delete []zBuffer;
    return 0;
}


