#pragma once

#include <Eigen/Core>

class VertexAttributes
{
public:
    VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
    {
        position << x, y, z, w;
        normal << 1,2,3,4;
        specular << 4, 5, 6;
        diffuse << 7, 8, 9;
        color << 1, 2, 3;
        specular_exponent = 256.0;
        trafo << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    }

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes &a,
        const VertexAttributes &b,
        const VertexAttributes &c,
        const float alpha,
        const float beta,
        const float gamma)
    {
        VertexAttributes r;
        r.position = alpha * a.position + beta * b.position + gamma * c.position;
        r.normal = alpha * a.normal + beta * b.normal + gamma * c.normal;
        r.specular = alpha * a.specular + beta * b.specular + gamma * c.specular;
        r.diffuse = alpha * a.diffuse + beta * b.diffuse + gamma * c.diffuse;
        r.color = alpha * a.color + beta * b.color + gamma * c.color;
        r.trafo = alpha * a.trafo + beta * b.trafo + gamma * c.trafo;
        return r;
    }

    Eigen::Vector4f position;
    Eigen::Vector4f normal;
    Eigen::Vector3f specular;
    Eigen::Vector3f diffuse;
    Eigen::Vector3f color;
    Eigen::Matrix4f trafo;
    float specular_exponent;
};

class FragmentAttributes
{
public:
    FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
    {
        color << r, g, b, a;
    }

    Eigen::Vector4f color;
    Eigen::Vector4f position;
};

class FrameBufferAttributes
{
public:
    FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
    {
        color << r, g, b, a;
        depth = 2;
    }

    Eigen::Matrix<uint8_t, 4, 1> color;
    float depth = 2;
};

class UniformAttributes
{
public:
    Eigen::Matrix4f camera_transform;
    Eigen::Matrix4f projection_transform;
    Eigen::Matrix4f perspective_transform;
    Eigen::Matrix4f P;
    Eigen::Vector3d ambient_light;
    std::vector<Eigen::Vector3d> light_position;
    std::vector<Eigen::Vector3d> light_colors;
    bool is_perspective;
};