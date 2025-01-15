// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// Utilities for the Assignment
#include "utils.h"
#include <math.h> 

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with perspective projection" << std::endl;

    const std::string filename("sphere_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);


    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);// -1, 1, 1
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);
    Vector3d sphere_origin(0, 0, 0);
    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            Vector3d ray_direction = RowVector3d(ray_origin(0)-camera_origin(0), ray_origin(1) - camera_origin(1), ray_origin(2) - camera_origin(2));
            ray_direction.normalize();
            //std::cout << "ray_direction" << ray_direction << std::endl;
            //Vector3d ray_direction = RowVector3d(0, 0, -1); //orthographic
            
            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for
            // orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;
            double a = ray_direction(0) * ray_direction(0) + ray_direction(1) * ray_direction(1) + ray_direction(2) * ray_direction(2);
            double b = 2 * (ray_direction(0) * (ray_origin(0) - sphere_origin(0)) + ray_direction(1) * (ray_origin(1) - sphere_origin(1)) + ray_direction(2) * (ray_origin(2) - sphere_origin(2)));
            double c = (((ray_origin(0) - sphere_origin(0)) * (ray_origin(0) - sphere_origin(0))) + ((ray_origin(1) - sphere_origin(1)) * (ray_origin(1) - sphere_origin(1))) + ((ray_origin(2) - sphere_origin(2)) * (ray_origin(2) - sphere_origin(2)))) - (sphere_radius*sphere_radius);
            double total = (b * b) - (4 * a * c);
            if (total >= 0)
            {
                // calculate the quadratic formula to solve for t with positive total
                // create a vector3d that is the intersection point defined by ray_origin + ray_direction * t
                //std::cout << "called" << std::endl;
                double t1 = ((b * -1) + sqrt(total)) / 2 * a;
                double t2 = ((b * -1) - sqrt(total)) / 2 * a;
                Vector3d ray_intersection;
                if (t1 < t2) {
                    ray_intersection = ray_origin + ray_direction * t1;
                }
                else {
                    ray_intersection = ray_origin + ray_direction * t2;
                }
                
                // The ray hit the sphere, compute the exact intersection point
                //Vector3d ray_intersection(
                //    ray_on_xy(0), ray_on_xy(1),
                //    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
                
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);
    Vector3d a = pgram_origin;
    Vector3d b = a + pgram_u;
    Vector3d c = a + pgram_v;


    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            Vector3d B = ray_origin - a;
            MatrixXd A_Plane = MatrixXd::Zero(3,3);
            A_Plane(0,0) = b(0)-a(0);
            A_Plane(0, 1) = c(0) - a(0);
            A_Plane(0, 2) = ray_direction(0) * -1;

            A_Plane(1, 0) = b(1) - a(1);
            A_Plane(1, 1) = c(1) - a(1);
            A_Plane(1, 2) = ray_direction(1) * -1;

            A_Plane(2, 0) = b(2) - a(2);
            A_Plane(2, 1) = c(2) - a(2);
            A_Plane(2, 2) = ray_direction(2) * -1;

            A_Plane = A_Plane.inverse();
            Vector3d solution = A_Plane * B; // (0) is u, (1) is v, (2) is t
            
            // TODO: Check if the ray intersects with the parallelogram
            if (solution(2) > 0 && solution(0) <= 1 && solution(0) >= 0 && solution(1) <= 1 && solution(1) >= 0)
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = ray_origin + ray_direction * solution(2);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);
    Vector3d a = pgram_origin;
    Vector3d b = a + pgram_u;
    Vector3d c = a + pgram_v;

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            Vector3d ray_direction = RowVector3d(ray_origin(0) - camera_origin(0), ray_origin(1) - camera_origin(1), ray_origin(2) - camera_origin(2));
            ray_direction.normalize();

            Vector3d B = ray_origin - a;
            MatrixXd A_Plane = MatrixXd::Zero(3, 3);
            A_Plane(0, 0) = b(0) - a(0);
            A_Plane(0, 1) = c(0) - a(0);
            A_Plane(0, 2) = ray_direction(0) * -1;

            A_Plane(1, 0) = b(1) - a(1);
            A_Plane(1, 1) = c(1) - a(1);
            A_Plane(1, 2) = ray_direction(1) * -1;

            A_Plane(2, 0) = b(2) - a(2);
            A_Plane(2, 1) = c(2) - a(2);
            A_Plane(2, 2) = ray_direction(2) * -1;

            A_Plane = A_Plane.inverse();
            Vector3d solution = A_Plane * B; // (0) is u, (1) is v, (2) is t

            // TODO: Check if the ray intersects with the parallelogram
            if (solution(2) > 0 && solution(0) <= 1 && solution(0) >= 0 && solution(1) <= 1 && solution(1) >= 0)
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = ray_origin + ray_direction * solution(2);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd R = MatrixXd::Zero(800, 800); // Store the red
    MatrixXd G = MatrixXd::Zero(800, 800); // Store the green
    MatrixXd B = MatrixXd::Zero(800, 800); // Store the blue
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y

    const Vector3d image_origin(-1, 1, 1);// -1, 1, 1
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);
    

    //Sphere setup
    Vector3d sphere_origin(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            Vector3d ray_direction = RowVector3d(ray_origin(0) - camera_origin(0), ray_origin(1) - camera_origin(1), ray_origin(2) - camera_origin(2));
            ray_direction.normalize();

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;
            double a = ray_direction(0) * ray_direction(0) + ray_direction(1) * ray_direction(1) + ray_direction(2) * ray_direction(2);
            double b = 2 * (ray_direction(0) * (ray_origin(0) - sphere_origin(0)) + ray_direction(1) * (ray_origin(1) - sphere_origin(1)) + ray_direction(2) * (ray_origin(2) - sphere_origin(2)));
            double c = (((ray_origin(0) - sphere_origin(0)) * (ray_origin(0) - sphere_origin(0))) + ((ray_origin(1) - sphere_origin(1)) * (ray_origin(1) - sphere_origin(1))) + ((ray_origin(2) - sphere_origin(2)) * (ray_origin(2) - sphere_origin(2)))) - (sphere_radius * sphere_radius);
            double total = (b * b) - (4 * a * c);

            if (total >= 0)
            {
                // The ray hit the sphere, compute the exact intersection point
                double t1 = ((b * -1) + sqrt(total)) / 2 * a;
                double t2 = ((b * -1) - sqrt(total)) / 2 * a;
                Vector3d ray_intersection;
                if (t1 < t2) {
                    ray_intersection = ray_origin + ray_direction * t1;
                }
                else {
                    ray_intersection = ray_origin + ray_direction * t2;
                }

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();
                Vector3d Li = (light_position - ray_intersection).normalized();
                Vector3d Vi = (camera_origin - ray_intersection).normalized();
                Vector3d Hi = (Li + Vi).normalized();

                // TODO: Add shading parameter here
                
                const double diffuse = std::max(0.0, Li.dot(ray_normal));
                const double specular = std::pow(std::max(0.0, Hi.dot(ray_normal)), specular_exponent);


                // Simple diffuse model
                C(i, j) = ambient + diffuse + specular;
                R(i, j) = ambient + diffuse_color(0)* diffuse + specular_color(0) * specular;
                G(i, j) = ambient + diffuse_color(1) * diffuse + specular_color(1) * specular;
                B(i, j) = ambient + diffuse_color(2) * diffuse + specular_color(2) * specular;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;

            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
