////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

//Maximum number of recursive calls
const int max_bounce = 5;
int bounce = 0;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Camera settings
double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const bool is_brute = false;
double offset = 0.0;
Vector3d camera_position(0, 0, 2);


// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

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
    //Loads file
    std::ifstream in(mesh_filename);
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

    //setup tree
    bvh = AABBTree(vertices, facets);

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

////////////////////////////////////////////////////////////////////////////////
// BVH Code
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
            N = (ray_intersection - sphere_center).normalized();
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

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}
//vertices, faces
AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    //std::cout << V << std::endl << std::endl;
    //std::cout << F << std::endl;

    Node root;

    // check for base case.

    if (false) {

    }
    else {
        // split triangles in two
        Node current;


        //sort the triangles to above and below chosen axis

        //call function recursivly on left and right part of list
    }

    /*for (int i = 0; i < facets.rows(); i += 1)
    {

        Vector3d a_vertex(V(F(i, 0), 0) - 0.5, V(F(i, 0), 1) - 0.5, V(F(i, 0), 2) - 0.5);
        Vector3d b_vertex(V(F(i, 1), 0) - 0.5, V(F(i, 1), 1) - 0.5, V(F(i, 1), 2) - 0.5);
        Vector3d c_vertex(V(F(i, 2), 0) - 0.5, V(F(i, 2), 1) - 0.5, V(F(i, 2), 2) - 0.5);
        //std::cout << a_vertex << std::endl << b_vertex << std::endl << c_vertex << std::endl << std::endl;

    }
    Node current;
    current.left = 0;*/
    
    // TODO

    // Top-down approach.
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a_vertex, const Vector3d &b_vertex, const Vector3d &c_vertex, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    const Vector3d pgram_origin = a_vertex;
    const Vector3d A = b_vertex;
    const Vector3d B = c_vertex;
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

    if (solution(2) > 0 &&  solution(0) >= 0 && solution(1) >= 0 && (solution(0) + solution(1)) <= 1)
    {
        //TODO set the correct intersection point, update p and N to the correct values
        p = ray_origin + ray_direction * solution(2);
        N = pgram_u.cross(pgram_v).normalized();
        return solution(2);
    }




    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    Vector3d box_max(box.corner(box.TopRightCeil));
    Vector3d box_min(box.corner(box.BottomLeft));
    double min_x = box_min(0);
    double min_y = box_min(1);
    double min_z = box_min(2);
    double max_x = box_max(0);
    double max_y = box_max(1);
    double max_z = box_max(2);

    double min_x_intersection, max_x_intersection, max_y_intersection, max_z_intersection, min_y_intersection, min_z_intersection;

    // get max and min x intersections
    min_x_intersection = (min_x - ray_origin(0)) / ray_direction(0);
    max_x_intersection = (max_x - ray_origin(0)) / ray_direction(0);

    if (min_x_intersection > max_x_intersection) {
        std::swap(min_x_intersection, max_x_intersection);
    }

        
    // get max and min y intersections
    min_y_intersection = (min_y - ray_origin(1)) / ray_direction(1);
    max_y_intersection = (max_y - ray_origin(1)) / ray_direction(1);
    if (min_y_intersection > max_y_intersection) {
        std::swap(min_y_intersection, max_y_intersection);
    }


    // compute whether intersection point falls in bounds
    if ((min_x_intersection > max_y_intersection) || (min_y_intersection > max_x_intersection)){
        return false;
    }

    if (min_y_intersection > min_x_intersection) {
        min_x_intersection = min_y_intersection;
    }

    if (max_y_intersection < max_x_intersection) {
        max_x_intersection = max_y_intersection;
    }

    // get max and min z intersections
    min_z_intersection = (min_z - ray_origin(2)) / ray_direction(2);
    max_z_intersection = (max_z - ray_origin(2)) / ray_direction(2);

    if (min_z_intersection > max_z_intersection) {
        std::swap(min_z_intersection, max_z_intersection);
    }


    if ((min_x_intersection > max_z_intersection) || (min_z_intersection > max_x_intersection))
        return false;

    if (min_z_intersection > min_x_intersection)
        min_x_intersection = min_z_intersection;

    if (max_z_intersection < max_x_intersection)
        max_x_intersection = max_z_intersection;

    return true;
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    double closest_t = std::numeric_limits<double>::max(); //closest t is "+ infinity"
    bool flag = false;
    if (is_brute)
    {
        for (int i = 0; i < facets.rows(); i += 1)
        {
            //returns t and writes on tmp_p and tmp_N

            Vector3d a_vertex(vertices(facets(i, 0), 0) - offset, vertices(facets(i, 0), 1) - offset, vertices(facets(i, 0), 2) - offset);
            Vector3d b_vertex(vertices(facets(i, 1), 0) - offset, vertices(facets(i, 1), 1) - offset, vertices(facets(i, 1), 2) - offset);
            Vector3d c_vertex(vertices(facets(i, 2), 0) - offset, vertices(facets(i, 2), 1) - offset, vertices(facets(i, 2), 2) - offset);
            //std::cout << "Value of i" << i << std::endl;
            //std::cout<< "A:\n " << a_vertex << "\nB: " << b_vertex << "\nC: " << c_vertex << std::endl;

            const double t = ray_triangle_intersection(ray_origin, ray_direction, a_vertex, b_vertex, c_vertex, tmp_p, tmp_N);
            //We have intersection
            if (t >= 0)
            {
                //std::cout << "Hit" << std::endl;
                //The point is before our current closest t
                if (t < closest_t)
                {
                    closest_t = t;
                    p = tmp_p;
                    N = tmp_N;
                    flag = true;
                }

            }


        }

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
                    closest_t = t;
                    p = tmp_p;
                    N = tmp_N;
                    flag = true;
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
                    closest_t = t;
                    p = tmp_p;
                    N = tmp_N;
                    flag = true;
                }
            }
        }

    }
    else {

        // faster version that checks to see if inside box before checking for triangles. basically using ray_box_intersection to speed up without AABB Tree
        for (int i = 0; i < facets.rows(); i += 1) {
            Vector3d a_vertex(vertices(facets(i, 0), 0) - offset, vertices(facets(i, 0), 1) - offset, vertices(facets(i, 0), 2) - offset);
            Vector3d b_vertex(vertices(facets(i, 1), 0) - offset, vertices(facets(i, 1), 1) - offset, vertices(facets(i, 1), 2) - offset);
            Vector3d c_vertex(vertices(facets(i, 2), 0) - offset, vertices(facets(i, 2), 1) - offset, vertices(facets(i, 2), 2) - offset);
            AlignedBox3d box = bbox_from_triangle(a_vertex, b_vertex, c_vertex);
            bool flag2 = ray_box_intersection(ray_origin, ray_direction, box);
            if (flag2) {
                const double t = ray_triangle_intersection(ray_origin, ray_direction, a_vertex, b_vertex, c_vertex, tmp_p, tmp_N);
                //We have intersection
                if (t >= 0)
                {
                    //std::cout << "Hit" << std::endl;
                    //The point is before our current closest t
                    if (t < closest_t)
                    {
                        closest_t = t;
                        p = tmp_p;
                        N = tmp_N;
                        flag = true;
                    }

                }
            }
        }


    }


    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.

    return flag;
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
    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (nearest_object == true) {

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

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
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
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];


        Vector4d diff_color = obj_diffuse_color;

        // TODO: Add shading parameters

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();

        // TODO: Shoot a shadow ray to determine if the light should affect the intersection point and call is_light_visible
        bool visible = is_light_visible(p + (0.0001 * Li), Li, light_position);
        if (visible == false) {
            continue;
        }

        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // TODO: Compute the color of the reflected ray and add its contribution to the current point color.
    // use refl_color
    Vector4d refl_color = obj_reflection_color;
    Vector4d reflection_color(0, 0, 0, 0);
    Vector3d reflect_direction = ray_direction - 2 * (ray_direction.dot(N)) * N;
    if (bounce < max_bounce) {
        bounce += 1;
        reflection_color = shoot_ray(p + (0.0001 * reflect_direction), reflect_direction, max_bounce);
        reflection_color(0) = reflection_color(0) * refl_color(0);
        reflection_color(1) = reflection_color(1) * refl_color(1);
        reflection_color(2) = reflection_color(2) * refl_color(2);
        reflection_color(3) = reflection_color(3) * refl_color(3);
        //std::cout << "Colour:" << std::endl << reflection_color << std::endl;
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    // change the scene to allow the spheres and plane to be see more easily
    if (is_brute) {
        offset = 0.5;
        camera_position(2) = 5;
        focal_length = 10;

    }

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = std::tan(field_of_view / 2) * focal_length;// 1;
    double image_x = double(w / h) * image_y;// 1;


    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);


    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();


            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }
            bounce = 0;
            const Vector4d C = shoot_ray(ray_origin, ray_direction,max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
        //std::cout << "Progress:" << float(i) / float(w) * 100 << "%" << std::endl;
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
