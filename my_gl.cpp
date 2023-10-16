//
// Created by galacy on 2023/10/14.
//
#include "geometry.h"
#include "tgaimage.h"
#include "my_gl.h"


IShader::~IShader() {}

Matrix Projection;
Matrix Viewport;
Matrix ModelView;

void viewport(int x, int y, int width, int height) {
    Viewport = Matrix::identity();

    Viewport[0][3] = (x + width) / 2.f;
    Viewport[1][3] = (y + height) / 2.f;
    Viewport[2][3] = 255.f/ 2.f;

    Viewport[0][0] = width / 2.f;
    Viewport[1][1] = height / 2.f;
    Viewport[2][2] = 255.f/ 2.f;
}

void projection(float coeff) {
    Projection = Matrix::identity();

    Projection[3][2] = coeff;
}

void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye-center).normalize();
    Vec3f x = cross(up,z).normalize();
    Vec3f y = cross(z,x).normalize();

    Matrix translation = Matrix::identity();
    Matrix rotation = Matrix::identity();
    ModelView = Matrix::identity();
    for (int i=0; i<3; i++) {
        rotation[0][i] = x[i];
        rotation[1][i] = y[i];
        rotation[2][i] = z[i];

        translation[i][3] = -center[i];
    }

    ModelView = rotation * translation;
}

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec4f* pts, IShader& shader, TGAImage& image, float* zbuffer) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j] / pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j] / pts[i][3]);
        }
    }

    Vec2i P;
    TGAColor color;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f bc = barycentric(proj<2>(pts[0] / pts[0][3]), proj<2>(pts[1] / pts[1][3]), proj<2>(pts[2] / pts[2][3]),
                                   proj<2>(P));
            float z = bc.x * pts[0][2] + bc.y * pts[1][2] + bc.z * pts[2][2];
            float w = bc.x * pts[0][3] + bc.y * pts[1][3] + bc.z * pts[2][3];
            int frag_depth = z / w;
            if (bc.x < 0 || bc.y < 0 || bc.z < 0 || zbuffer[P.x + P.y * image.get_width()] > frag_depth)
                continue;
            bool discard = shader.fragment(bc, color);
            if (!discard) {
                zbuffer[P.x + P.y * image.get_width()] = frag_depth;
                image.set(P.x, P.y, color);
            }
        }
    }
}
