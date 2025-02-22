
////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////

const std::string filename("raytrace.png");

//Camera settings
const double focal_length = 10;
const double field_of_view = 0.7854; //45 degrees
const double image_z = 5;
const bool is_perspective = true;
const bool is_linear = false; // used to determine if linear or cubic interpolation for perlin noise
const Vector3d camera_position(0, 0, 5);

//Maximum number of recursive calls
const int max_bounce = 5;
int bounce = 0;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.5, 0.1, 0.1, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);
const Vector4d obj_refraction_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    grid.resize(grid_size + 1);
    for (int i = 0; i < grid_size + 1; ++i)
    {
        grid[i].resize(grid_size + 1);
        for (int j = 0; j < grid_size + 1; ++j)
            grid[i][j] = Vector2d::Random().normalized();
    }

    //Spheres
    sphere_centers.emplace_back(10, 0, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(7, 0.05, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-8, 1.6, 1);
    sphere_radii.emplace_back(1);

    //parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

//We need to make this function visible
Vector4d shoot_ray(const Vector3d& ray_origin, const Vector3d& ray_direction, int max_bounce);

////////////////////////////////////////////////////////////////////////////////
// Perlin noise code
////////////////////////////////////////////////////////////////////////////////

// Function to linearly interpolate between a0 and a1
// Weight w should be in the range [0.0, 1.0]
double lerp(double a0, double a1, double w)
{
    assert(w >= 0);
    assert(w <= 1);
    //TODO implement linear and cubic interpolation
    if (is_linear) {
        return (1.0 - w) * a0 + w * a1;
    }
    else {
        return (a1 - a0) * (3.0 - w * 2.0) * w * w + a0;
    }
}

// Computes the dot product of the distance and gradient vectors.
double dotGridGradient(int ix, int iy, double x, double y)
{
    //TODO: Compute the distance vector
    float dx = x - float(ix);
    float dy = y - float(iy);

    //TODO: Compute and return the dot-product
    return dx * grid[iy][ix][0] + dy * grid[iy][ix][1];
}

// Compute Perlin noise at coordinates x, y
double perlin(double x, double y)
{
    //TODO: Determine grid cell coordinates x0, y0
    int x0 = int(x);
    int x1 = x0 + 1;
    int y0 = int(y);
    int y1 = y0 + 1;

    // Determine interpolation weights
    double sx = x - x0;
    double sy = y - y0;

    // Interpolate between grid point gradients
    double n0 = dotGridGradient(x0, y0, x, y);
    double n1 = dotGridGradient(x1, y0, x, y);

    double ix0 = lerp(n0, n1, sx);

    n0 = dotGridGradient(x0, y1, x, y);
    n1 = dotGridGradient(x1, y1, x, y);

    double ix1 = lerp(n0, n1, sx);
    double value = lerp(ix0, ix1, sy);

    return value;
}

Vector4d procedural_texture(const double tu, const double tv)
{
    assert(tu >= 0);
    assert(tv >= 0);

    assert(tu <= 1);
    assert(tv <= 1);

    //TODO: uncomment these lines once you implement the perlin noise
     const double color = (perlin(tu * grid_size, tv * grid_size) + 1) / 2;
     return Vector4d(0, color, 0, 0);

    //Example fo checkerboard texture
    //const double color = (int(tu * grid_size) + int(tv * grid_size)) % 2 == 0 ? 0 : 1;
    //return Vector4d(0, color, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////
//Compute the intersection between a ray and a sphere, return -1 if no intersection
double ray_sphere_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, int index, Vector3d& p, Vector3d& N)
{
    // TODO, implement the intersection between the ray and the sphere at index index.
    //return t or -1 if no intersection

    const Vector3d sphere_center = sphere_centers[index];
    const double sphere_radius = sphere_radii[index];

    double a = ray_direction.dot(ray_direction);
    double b = 2 * ray_direction.dot(ray_origin - sphere_center);
    double c = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - (sphere_radius * sphere_radius);
    double total = (b * b) - (4 * a * c);
    double t = 0;


    if (total < 0)
    {
        return -1;
    }
    else
    {
        double t1 = ((b * -1) + sqrt(total)) / (2 * a);
        double t2 = ((b * -1) - sqrt(total)) / (2 * a);
        Vector3d ray_intersection;
        if (t1 < t2 && t1 >= 0) {
            
            ray_intersection = ray_origin + ray_direction * t1;
            p = ray_intersection;
            N = (ray_intersection-sphere_center).normalized();
            return t1;
        }
        else if (t2 >= 0) {
            ray_intersection = ray_origin + ray_direction * t2;
            p = ray_intersection;
            N = (ray_intersection - sphere_center).normalized();
            return t2;
        }
    }

    return -1;
}

//Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, int index, Vector3d& p, Vector3d& N)
{
    // TODO, implement the intersection between the ray and the parallelogram at index index.
    //return t or -1 if no intersection

    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;
    Vector3d a = pgram_origin;
    Vector3d b = a + pgram_u;
    Vector3d c = a + pgram_v;

    Vector3d Beta = ray_origin - a;
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
    //std::cout << "Value of A_Plane inverse" << std::endl << A_Plane << std::endl;
    Vector3d solution = A_Plane * Beta; // (0) is u, (1) is v, (2) is t



    if (solution(2) > 0 && solution(0) <= 1 && solution(0) >= 0 && solution(1) <= 1 && solution(1) >= 0)
    {
        //TODO set the correct intersection point, update p and N to the correct values
        p = ray_origin + ray_direction * solution(2);
        N = pgram_v.cross(pgram_u).normalized();
        return solution(2);
    }

    

    return -1;
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
int find_nearest_object(const Vector3d& ray_origin, const Vector3d& ray_direction, Vector3d& p, Vector3d& N)
{
    // Find the object in the scene that intersects the ray first
    // we store the index and the 'closest_t' to their expected values
    int closest_index = -1;
    double closest_t = std::numeric_limits<double>::max(); //closest t is "+ infinity"

    Vector3d tmp_p, tmp_N;
    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    for (int i = 0; i < parallelograms.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = sphere_centers.size() + i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    return closest_index;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

//Checks if the light is visible
bool is_light_visible(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& light_position)
{
    // TODO: Determine if the light is visible here
    // Use find_nearest_object
    Vector3d p, N; // p is the point of intersection, N is the normal at that point
    const int nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (nearest_object >= 0) {

        // check to see if nearest object is in front or behind the light
        Vector3d light_vector = light_position - ray_origin;
        double light_distance = sqrt(light_vector(0) * light_vector(0) + light_vector(1) * light_vector(1) + light_vector(2) * light_vector(2));

        Vector3d object_vector = p - ray_origin;
        double object_distance = sqrt(object_vector(0) * object_vector(0) + object_vector(1) * object_vector(1) + object_vector(2) * object_vector(2));


        if (object_distance > light_distance) {
            return true;
        }
        return false;
    }

    return true;
}

Vector4d shoot_ray(const Vector3d& ray_origin, const Vector3d& ray_direction, int max_bounce)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const int nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (nearest_object < 0)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d& light_position = light_positions[i];
        const Vector4d& light_color = light_colors[i];

        const Vector3d Li = (light_position - p).normalized();
        Vector3d Vi = (camera_position - p).normalized();
        Vector3d Hi = (Li + Vi).normalized();

        // TODO: Shoot a shadow ray to determine if the light should affect the intersection point and call is_light_visible
        bool visible = is_light_visible(p + (0.0001 * Li), Li, light_position);
        if (visible == false) {
            continue;
        }

        Vector4d diff_color = obj_diffuse_color;

        if (nearest_object == 4)
        {
            //Compute UV coodinates for the point on the sphere
            const double x = p(0) - sphere_centers[nearest_object][0];
            const double y = p(1) - sphere_centers[nearest_object][1];
            const double z = p(2) - sphere_centers[nearest_object][2];
            const double tu = acos(z / sphere_radii[nearest_object]) / 3.1415;
            const double tv = (3.1415 + atan2(y, x)) / (2 * 3.1415);

            diff_color = procedural_texture(tu, tv);
        }

        // TODO: Add shading parameters

        // Diffuse contribution
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution, use obj_specular_color
        const Vector4d specular = obj_specular_color * std::pow(std::max(0.0, Hi.dot(N)), obj_specular_exponent);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    Vector4d refl_color = obj_reflection_color;
    if (nearest_object == 4)
    {
        refl_color = Vector4d(0.5, 0.5, 0.5, 0);
    }
    // TODO: Compute the color of the reflected ray and add its contribution to the current point color.
    // use refl_color
    Vector4d reflection_color(0, 0, 0, 0);
    Vector3d reflect_direction = ray_direction - 2 * (ray_direction.dot(N)) * N;
    if (bounce < max_bounce) {
        bounce += 1;
        reflection_color = shoot_ray(p + (0.0001 * reflect_direction),reflect_direction,max_bounce);
        reflection_color(0) = reflection_color(0) * refl_color(0);
        reflection_color(1) = reflection_color(1) * refl_color(1);
        reflection_color(2) = reflection_color(2) * refl_color(2);
        reflection_color(3) = reflection_color(3) * refl_color(3);
        //std::cout << "Colour:" << std::endl << reflection_color << std::endl;
    }


    // TODO: Compute the color of the refracted ray and add its contribution to the current point color.
    //       Make sure to check for total internal reflection before shooting a new ray.
    Vector4d refraction_color(0, 0, 0, 0);

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color + refraction_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 800;
    int h = 400;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    double image_y = std::tan(field_of_view/2) * focal_length; //TODO: compute the correct pixels size
    double image_x = w/h * image_y; //TODO: compute the correct pixels size

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, -image_z);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            // TODO: Implement depth of field
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // TODO: Perspective camera
                ray_origin = camera_position;
                ray_direction = pixel_center - camera_position;
            }
            else
            {
                // Orthographic camera
                ray_origin = camera_position + Vector3d(pixel_center[0], pixel_center[1], 0);
                ray_direction = Vector3d(0, 0, -1);
            }
            bounce = 0;
            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
        //std::cout << "Progress:" << float(i) /float(w) * 100 << "%" << std:: endl;
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}