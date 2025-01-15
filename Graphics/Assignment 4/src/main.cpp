// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5; //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = false;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    //TODO: setup uniform

    uniform.light_position = light_positions;
    uniform.light_colors = light_colors;
    uniform.ambient_light = ambient_light;

    //TODO: setup camera, compute w, u, v
    const Vector3d w = -1*camera_gaze.normalized();
    const Vector3d u = camera_top.cross(w).normalized();
    const Vector3d v = w.cross(u);
    //TODO: compute the camera transformation
    Matrix4f camera_transform = Matrix4f::Zero(4,4);
    camera_transform(0, 0) = (float)u(0);
    camera_transform(1, 0) = (float)u(1);
    camera_transform(2, 0) = (float)u(2);
    camera_transform(0, 1) = (float)v(0);
    camera_transform(1, 1) = (float)v(1);
    camera_transform(2, 1) = (float)v(2);
    camera_transform(0, 2) = (float)w(0);
    camera_transform(1, 2) = (float)w(1);
    camera_transform(2, 2) = (float)w(2);
    camera_transform(3, 3) = 1.0;

    camera_transform(0, 3) = (float)camera_position(0);
    camera_transform(1, 3) = (float)camera_position(1);
    camera_transform(2, 3) = (float)camera_position(2);
    camera_transform(3, 3) = 1.0;
    //std::cout << camera_transform << std::endl;
    camera_transform = camera_transform.inverse().eval();
    //std::cout << camera_transform << std::endl;

    //TODO: setup projection matrix
    float n = (float)( - 1 * near_plane); // multiply by -1 since z direction is negative
    float f = (float) ( - 1 * far_plane);
    float height = (float)(std::tan(field_of_view / 2) * near_plane);
    float width = (float)(aspect_ratio * height);
    float l = (float)(width * -1);
    float r = (float)(width);
    float b = (float)(height *-1);
    float t = (float)(height);
    /*std::cout << "Near " << n << std::endl;
    std::cout << "Far " << f << std::endl;
    std::cout << "Left " << l << std::endl;
    std::cout << "Right " << r<< std::endl;
    std::cout << "Top " << t << std::endl;
    std::cout << "Bottom " << b << std::endl;*/
    Matrix4f projection_transform = Matrix4f::Zero(4, 4);
    projection_transform(0, 0) = (float)(2/(r-l));
    projection_transform(1, 1) = (float)(2/(t-b));
    projection_transform(2, 2) = (float)(2/(n-f));
    projection_transform(0,3) = (float)( - 1 * ((r + l) / (r - l)));
    projection_transform(1, 3) = (float)( - 1 * ((t + b) / (t - b)));
    projection_transform(2, 3) = (float)( - 1 * ((n + f) / (n - f)));
    projection_transform(3, 3) = 1.0;
    //std::cout << projection_transform << std::endl;


    Matrix4f P = projection_transform * camera_transform;
    uniform.P = P;
    Matrix4f perspective;
    perspective << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    uniform.projection_transform = projection_transform;
    uniform.camera_transform = camera_transform;
    if (is_perspective)
    {
        //TODO setup prespective camera
        perspective = Matrix4f::Zero(4, 4);
        perspective(0, 0) = n;
        perspective(1, 1) = n;
        perspective(2, 2) = n + f;
        perspective(2,3) = -f * n;
        perspective(3,2) = 1;
        //std::cout << perspective << std::endl;
    }
    else
    {
    }
    uniform.is_perspective = is_perspective;
    uniform.perspective_transform = perspective;
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.P * va.position;
        //std::cout << uniform.P << std::endl;
        //std::cout << va.position << std::endl;
        //std::cout << out.position << std::endl;

        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets


    for (int i = 0; i < facets.rows(); i += 1) {
        Vector4f a_vertex((float)vertices(facets(i, 0), 0), (float)vertices(facets(i, 0), 1),(float) vertices(facets(i, 0), 2),1);
        Vector4f b_vertex((float)vertices(facets(i, 1), 0), (float)vertices(facets(i, 1), 1), (float)vertices(facets(i, 1), 2),1);
        Vector4f c_vertex((float)vertices(facets(i, 2), 0) , (float)vertices(facets(i, 2), 1), (float)vertices(facets(i, 2), 2),1);

        vertex_attributes.push_back(VertexAttributes(a_vertex(0),a_vertex(1),a_vertex(2)));
        vertex_attributes.push_back(VertexAttributes(b_vertex(0), b_vertex(1), b_vertex(2)));
        vertex_attributes.push_back(VertexAttributes(c_vertex(0), c_vertex(1), c_vertex(2)));

        
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;
    res(0, 0) = std::cos(alpha);
    res(1, 0) = 0;
    res(2, 0) = -1*std::sin(alpha);
    res(3, 0) = 0;

    res(0, 1) = 0;
    res(1, 1) = 1;
    res(2, 1) = 0;
    res(3, 1) = 0;

    res(0, 2) = std::sin(alpha);
    res(1, 2) = 0;
    res(2, 2) = std::cos(alpha);
    res(3, 2) = 0;

    res(0, 3) = 0;
    res(1, 3) = 0;
    res(2, 3) = 0;
    res(3, 3) = 1;

    //std::cout << "rotation matrix" << std::endl;
    //std::cout << res << std::endl;


    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;
        Vector4f rotate_position;
        rotate_position = va.trafo * va.position;
        out.position = uniform.P * rotate_position;
        out.normal = uniform.P * va.normal;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: generate the vertex attributes for the edges and rasterize the lines
    for (int i = 0; i < facets.rows(); i += 1) {
        Vector4f a_vertex((float)vertices(facets(i, 0), 0), (float)vertices(facets(i, 0), 1), (float)vertices(facets(i, 0), 2), 1);
        Vector4f b_vertex((float)vertices(facets(i, 1), 0), (float)vertices(facets(i, 1), 1), (float)vertices(facets(i, 1), 2), 1);
        Vector4f c_vertex((float)vertices(facets(i, 2), 0), (float)vertices(facets(i, 2), 1), (float)vertices(facets(i, 2), 2), 1);

        VertexAttributes a(a_vertex(0), a_vertex(1), a_vertex(2));
        VertexAttributes b(b_vertex(0), b_vertex(1), b_vertex(2));
        VertexAttributes c(c_vertex(0), c_vertex(1), c_vertex(2));

        a.trafo << trafo.cast<float>();
        b.trafo << trafo.cast<float>();
        c.trafo << trafo.cast<float>();

        vertex_attributes.push_back(a);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(a);
        vertex_attributes.push_back(c);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(c);
    }

     
    //TODO: use the transformation matrix

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal
        //TODO: compute the correct lighting
        VertexAttributes out;
        Vector4f rotate_position;
        rotate_position = va.trafo * va.position;
        if (uniform.is_perspective == true) {
            out.position = uniform.camera_transform * rotate_position;
            out.position = uniform.perspective_transform * out.position;
            out.position = uniform.projection_transform * out.position;
        }
        else {
            out.position = uniform.P * rotate_position;
        }
        out.normal = uniform.P * va.normal;
        Vector3f lights_color(0, 0, 0);

        for (int i = 0; i < uniform.light_position.size(); ++i) {
            const Vector3f& light_position = uniform.light_position[i].cast<float>();
            const Vector3f& light_color = uniform.light_colors[i].cast<float>();
            Vector3f p(va.position(0), va.position(1), va.position(2));
            Vector3f N(va.normal(0), va.normal(1), va.normal(2));

            //std::cout << p << std::endl;
            //std::cout << N << std::endl;

            const Vector3f Li = (light_position - p).normalized();

            const Vector3f diffuse = va.diffuse * std::max(Li.dot(N), 0.0f);
            const Vector3f D = light_position - p;

            const Vector3f Hi = (Li - Vector3f(0, 0, 1)).normalized();
            const Vector3f specular = va.specular * std::pow(std::max(N.dot(Hi), 0.0f), va.specular_exponent);

            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }
        lights_color = uniform.ambient_light.cast<float>() + lights_color;

        //std::cout <<"Light color:" << lights_color << std::endl;
        out.color = lights_color;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment

        FragmentAttributes new_out(va.color(0), va.color(1), va.color(2));
        if (uniform.is_perspective){
            new_out.position = va.position;
        }
        else {
            new_out.position << va.position(0), va.position(1), va.position(2) * -1 , va.position(3);
        }

        return new_out;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        if (fa.position[2] < previous.depth)
        {
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
            return out;
        }
        else
            return previous;
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: compute the normals
    //TODO: set material colors

    for (int i = 0; i < facets.rows(); i += 1) {
        Vector4f a_vertex((float)vertices(facets(i, 0), 0), (float)vertices(facets(i, 0), 1), (float)vertices(facets(i, 0), 2), 1);
        Vector4f b_vertex((float)vertices(facets(i, 1), 0), (float)vertices(facets(i, 1), 1), (float)vertices(facets(i, 1), 2), 1);
        Vector4f c_vertex((float)vertices(facets(i, 2), 0), (float)vertices(facets(i, 2), 1), (float)vertices(facets(i, 2), 2), 1);

        VertexAttributes a(a_vertex(0), a_vertex(1), a_vertex(2));
        VertexAttributes b(b_vertex(0), b_vertex(1), b_vertex(2));
        VertexAttributes c(c_vertex(0), c_vertex(1), c_vertex(2));

        a.diffuse << (float)obj_diffuse_color(0), (float)obj_diffuse_color(1), (float)obj_diffuse_color(2);
        a.specular << (float)obj_specular_color(0), (float)obj_specular_color(1), (float)obj_specular_color(2);
        a.specular_exponent = obj_specular_exponent;
        b.diffuse << (float)obj_diffuse_color(0), (float)obj_diffuse_color(1), (float)obj_diffuse_color(2);
        b.specular << (float)obj_specular_color(0), (float)obj_specular_color(1), (float)obj_specular_color(2);
        b.specular_exponent = obj_specular_exponent;
        c.diffuse << (float)obj_diffuse_color(0), (float)obj_diffuse_color(1), (float)obj_diffuse_color(2);
        c.specular << (float)obj_specular_color(0), (float)obj_specular_color(1), (float)obj_specular_color(2);
        c.specular_exponent = obj_specular_exponent;

        Vector3f u(b.position(0) - a.position(0), b.position(1) - a.position(1), b.position(2) - a.position(2));
        Vector3f v(c.position(0) - a.position(0), c.position(1) - a.position(1), c.position(2) - a.position(2));
        Vector3f temp_normal = u.cross(v).normalized();
        Vector4f normal(temp_normal(0), temp_normal(1), temp_normal(2), 0);

        a.normal << normal;
        b.normal << normal;
        c.normal << normal;

        a.trafo << trafo.cast<float>();
        b.trafo << trafo.cast<float>();
        c.trafo << trafo.cast<float>();

        /*std::cout << a.position << std::endl;
        std::cout << b.position << std::endl;
        std::cout << c.position << std::endl;
        std::cout << u << std::endl;
        std::cout << v << std::endl;
        std::cout << a.normal << std::endl;
        std::cout << a.diffuse << std::endl;
        std::cout << a.specular << std::endl;*/

        vertex_attributes.push_back(a);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(c);


    }


    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);
    vector<Vector3f> normal;

    // set all normals to (0,0,0). will be 3 times as big as needed, but should be fine
    for (int i = 0; i < vertices.size(); i++) {
        normal.emplace_back(0, 0, 0);
    }
    // sum up all the normals for each vertex
    for (int i = 0; i < facets.rows(); i += 1) {
        int vertex_index1 = facets(i, 0);
        int vertex_index2 = facets(i, 1);
        int vertex_index3 = facets(i, 2);
        Vector4f a_vertex((float)vertices(vertex_index1, 0), (float)vertices(vertex_index1, 1), (float)vertices(vertex_index1, 2), 1);
        Vector4f b_vertex((float)vertices(vertex_index2, 0), (float)vertices(vertex_index2, 1), (float)vertices(vertex_index2, 2), 1);
        Vector4f c_vertex((float)vertices(vertex_index3, 0), (float)vertices(vertex_index3, 1), (float)vertices(vertex_index3, 2), 1);

        Vector3f u(b_vertex(0) - a_vertex(0), b_vertex(1) - a_vertex(1), b_vertex(2) - a_vertex(2));
        Vector3f v(c_vertex(0) - a_vertex(0), c_vertex(1) - a_vertex(1), c_vertex(2) - a_vertex(2));
        Vector3f temp_normal = u.cross(v).normalized();
        normal[vertex_index1] += temp_normal;
        normal[vertex_index2] += temp_normal;
        normal[vertex_index3] += temp_normal;
    }

    // normalize all the normal vectors
    for (int i = 0; i < vertices.size(); i++) {
        normal[i].normalize();
    }

    //TODO: compute the vertex normals as vertex normal average

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: create vertex attributes
    //TODO: set material colors

    for (int i = 0; i < facets.rows(); i += 1) {
        Vector4f a_vertex((float)vertices(facets(i, 0), 0), (float)vertices(facets(i, 0), 1), (float)vertices(facets(i, 0), 2), 1);
        Vector4f b_vertex((float)vertices(facets(i, 1), 0), (float)vertices(facets(i, 1), 1), (float)vertices(facets(i, 1), 2), 1);
        Vector4f c_vertex((float)vertices(facets(i, 2), 0), (float)vertices(facets(i, 2), 1), (float)vertices(facets(i, 2), 2), 1);

        VertexAttributes a(a_vertex(0), a_vertex(1), a_vertex(2));
        VertexAttributes b(b_vertex(0), b_vertex(1), b_vertex(2));
        VertexAttributes c(c_vertex(0), c_vertex(1), c_vertex(2));

        a.diffuse << (float)obj_diffuse_color(0), (float)obj_diffuse_color(1), (float)obj_diffuse_color(2);
        a.specular << (float)obj_specular_color(0), (float)obj_specular_color(1), (float)obj_specular_color(2);
        a.specular_exponent = obj_specular_exponent;
        b.diffuse << (float)obj_diffuse_color(0), (float)obj_diffuse_color(1), (float)obj_diffuse_color(2);
        b.specular << (float)obj_specular_color(0), (float)obj_specular_color(1), (float)obj_specular_color(2);
        b.specular_exponent = obj_specular_exponent;
        c.diffuse << (float)obj_diffuse_color(0), (float)obj_diffuse_color(1), (float)obj_diffuse_color(2);
        c.specular << (float)obj_specular_color(0), (float)obj_specular_color(1), (float)obj_specular_color(2);
        c.specular_exponent = obj_specular_exponent;

        a.normal << normal[facets(i, 0)].cast<float>(),0;
        b.normal << normal[facets(i, 1)].cast<float>(),0;
        c.normal << normal[facets(i, 2)].cast<float>(),0;

        a.trafo << trafo.cast<float>();
        b.trafo << trafo.cast<float>();
        c.trafo << trafo.cast<float>();

        vertex_attributes.push_back(a);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(c);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);


}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer2(W, H);
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer3(W, H);
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer4(W, H);
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer5(W, H);
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer6(W, H);
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer7(W, H);
    vector<uint8_t> image;
    std::cout << "starting" << std::endl;

    std::cout << "simple render" << std::endl;
    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);
    
    std::cout << "wireframe render" << std::endl;
    wireframe_render(0, frameBuffer2);
    framebuffer_to_uint8(frameBuffer2, image);
    stbi_write_png("wireframe.png", frameBuffer2.rows(), frameBuffer2.cols(), 4, image.data(), frameBuffer2.rows() * 4);
    
    std::cout << "flat render" << std::endl;
    flat_shading(0, frameBuffer3);
    framebuffer_to_uint8(frameBuffer3, image);
    stbi_write_png("flat_shading.png", frameBuffer3.rows(), frameBuffer3.cols(), 4, image.data(), frameBuffer3.rows() * 4);

    std::cout << "pv render" << std::endl;
    pv_shading(0, frameBuffer4);
    framebuffer_to_uint8(frameBuffer4, image);
    stbi_write_png("pv_shading.png", frameBuffer4.rows(), frameBuffer4.cols(), 4, image.data(), frameBuffer4.rows() * 4);
    std::cout << "finishing" << std::endl;




    //TODO: add the animation
    
    
    const char* fileName = "wireframe_rotate.gif";
    int delay = 25;
    GifWriter g;
    GifBegin(&g, fileName, frameBuffer5.rows(), frameBuffer5.cols(), delay);
    std::cout << "writing wireframe_rotate" << std::endl;
    for (float i = 0; i < 6.2; i += 0.1)
    {
        frameBuffer5.setConstant(FrameBufferAttributes());
        wireframe_render(i, frameBuffer5);
        framebuffer_to_uint8(frameBuffer5, image);
        GifWriteFrame(&g, image.data(), frameBuffer5.rows(), frameBuffer5.cols(), delay);
        std::cout << "Progress: " << i / 6.2 * 100 << "%" << std::endl;
    }

    GifEnd(&g);

    const char* fileName1 = "flat_shading_rotate.gif";
    GifWriter g1;
    GifBegin(&g1, fileName1, frameBuffer6.rows(), frameBuffer6.cols(), delay);
    std::cout << "writing flat_shading_rotate" << std::endl;
    for (float i = 0; i < 6.2; i += 0.1)
    {
        frameBuffer6.setConstant(FrameBufferAttributes());
        flat_shading(i, frameBuffer6);
        framebuffer_to_uint8(frameBuffer6, image);
        GifWriteFrame(&g1, image.data(), frameBuffer6.rows(), frameBuffer6.cols(), delay);
        std::cout << "Progress: " << i / 6.2 *100<< "%" << std::endl;
    }

    GifEnd(&g1);

    const char* fileName2 = "pv_shading_rotate.gif";
    GifWriter g2;
    GifBegin(&g2, fileName2, frameBuffer7.rows(), frameBuffer7.cols(), delay);
    std::cout << "writing pv_shading_rotate" << std::endl;
    for (float i = 0; i < 6.2; i += 0.1)
    {
        frameBuffer7.setConstant(FrameBufferAttributes());
        pv_shading(i, frameBuffer7);
        framebuffer_to_uint8(frameBuffer7, image);
        GifWriteFrame(&g2, image.data(), frameBuffer7.rows(), frameBuffer7.cols(), delay);
        std::cout << "Progress: " << i / 6.2 *100<< "%" << std::endl;
    }

    GifEnd(&g2);


    return 0;
}
