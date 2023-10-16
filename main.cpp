#include <vector>
#include <iostream>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "my_gl.h"

Model *model = nullptr;
float* shadowbuffer = nullptr;
const int width = 800;
const int height = 800;

Vec3f light_dir(1,1,1);
Vec3f       eye(1,1,3);
Vec3f    center(0,0,0);
Vec3f        up(0,1,0);

//normal mapping shader without
struct Shader : public IShader {
    mat<2,3,float> varying_uv;
    mat<3 ,3, float> varying_tri;
    mat<4,4,float> uniform_M; //projection * ModelView
    mat<4,4,float> uniform_MIT; //(projection * ModelView).inver_transpose
    mat<4,4,float> uniform_MShadow; //transform framebuffer screen to shadowbuffer screen


    Shader(Matrix M, Matrix MIT, Matrix MS): uniform_M(M), uniform_MIT(MIT), uniform_MShadow(MS), varying_uv(), varying_tri() {}

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_vertex = embed<4>(model->vert(iface,nthvert));
        gl_vertex = Viewport * Projection * ModelView *gl_vertex;
        varying_tri.set_col(nthvert, proj<3>(gl_vertex / gl_vertex[3]));
        return gl_vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec4f sb_p = uniform_MShadow * embed<4>(varying_tri * bar); // corresponding point in the shadow buffer
        sb_p = sb_p / sb_p[3];
        int idx = int(sb_p[0]) + int(sb_p[1]) * width; //x + y*width
        float shadow = .3+.7*(shadowbuffer[idx]<sb_p[2]); // magic coeff to avoid z-fighting

        Vec2f uv = varying_uv * bar;
        Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize();
        Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize();
        Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
        float diff = std::max(0.f, n * l);
        TGAColor c = model->diffuse(uv);
        for(int i = 0; i < 3; i++)
            color[i] = std::min<float>(20 + c[i]*shadow*(1.2*diff + .6*spec),255);
        return false;
    }
};


struct DepthShader : public IShader {
    mat<3 ,3, float> varying_tri;

    DepthShader(): varying_tri() {}
    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_vertex = embed<4>(model->vert(iface,nthvert));
        gl_vertex = Viewport * Projection * ModelView * gl_vertex;
        varying_tri.set_col(nthvert, proj<3>(gl_vertex / gl_vertex[3]));
        return gl_vertex;
    }
    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f p = varying_tri * bar;
        color = TGAColor(255, 255, 255) * (p.z / depth);
        return false;
    }


};

int main(int argc , char** argv) {
    float* zbuffer = new float[width * height];
    shadowbuffer = new float[width * height];
    for (int i = width * height; i-- ;) {
        zbuffer[i] = shadowbuffer[i] = -std::numeric_limits<float>::max();
    }
    model = new Model("../obj/diablo3_pose.obj");
    light_dir.normalize();

    {
        TGAImage depth(width, height, TGAImage::RGB);
        lookat(light_dir, center, up);
        viewport(width/8, height/8, width*3/4, height*3/4);
        projection(0);

        DepthShader depthShader;
        Vec4f screen_coords[3];
        for (int i = 0; i < model->nfaces(); i++) {
            for (int j = 0; j < 3; j++) {
                screen_coords[j] = depthShader.vertex(i, j);
            }
            triangle(screen_coords, depthShader, depth, shadowbuffer);
        }
        depth.flip_vertically();
        depth.write_tga_file("depth.tga");
    }
    Matrix M = Viewport * Projection * ModelView;
    {
        TGAImage frame(width, height, TGAImage::RGB);
        lookat(eye, center, up);
        viewport(width/8, height/8, width*3/4, height*3/4);
        projection(-1.f / (eye - center).norm());

        Shader shader(ModelView, (Projection * ModelView).invert_transpose(), M * (Viewport * Projection * ModelView).invert());
        Vec4f screen_coords[3];
        for (int i = 0; i < model->nfaces(); i++) {
            for (int j = 0; j < 3; j++) {
                screen_coords[j] = shader.vertex(i, j);
            }
            triangle(screen_coords, shader, frame, zbuffer);
        }
        frame.flip_vertically();
        frame.write_tga_file("output.tga");
    }

    delete model;
    delete [] zbuffer;
    delete [] shadowbuffer;

    return 0;
}


