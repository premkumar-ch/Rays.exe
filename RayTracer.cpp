#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

using namespace std;

const int width = 2048;
const int height = 2048;

double inf = numeric_limits<double>::infinity();

enum class LIGHT_TYPE { ambient, directional, point };

struct Vec3 {
    float x, y, z;
};

struct Color {
    int r, g, b;
};

Color operator*(Color c, float intensity) {
    return {
        min(255, (int)(c.r * intensity)),
        min(255, (int)(c.g * intensity)),
        min(255, (int)(c.b * intensity))
    };
}

Color operator+(Color a, Color b) {
    return {
        min(255, a.r + b.r),
        min(255, a.g + b.g),
        min(255, a.b + b.b)
    };
}

Vec3 operator-(Vec3 v) {
    return { -v.x, -v.y, -v.z };
}

struct Sphere {
    Vec3 center;
    float radius;
    Color color;
    float reflective;
};

struct Light {
    LIGHT_TYPE type;
    float intensity;
    Vec3 position;
    Vec3 direction;
};

Vec3 vec_sub(Vec3 a, Vec3 b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

Vec3 vec_add(Vec3 a, Vec3 b) {
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

Vec3 vec_scale(Vec3 v, float t) {
    return { v.x * t, v.y * t, v.z * t };
}

float dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float length(Vec3 v) {
    return sqrt(dot(v, v));
}

Vec3 normalize(Vec3 v) {
    float l = length(v);
    return { v.x / l, v.y / l, v.z / l };
}

Vec3 reflect_ray(Vec3 R, Vec3 N) {
    return vec_sub(vec_scale(N, 2.0f * dot(N, R)), R);
}

vector<Color> image(width* height);

void put_pixel(int x, int y, Color color) {
    int px = x + width / 2;
    int py = height / 2 - y - 1;

    if (px < 0 || px >= width || py < 0 || py >= height)
        return;

    image[py * width + px] = color;
}

pair<float, float> intersect_ray(Vec3 O, Vec3 D, Sphere s) {
    Vec3 CO = vec_sub(O, s.center);

    float a = dot(D, D);
    float b = 2 * dot(CO, D);
    float c = dot(CO, CO) - s.radius * s.radius;

    float disc = b * b - 4 * a * c;

    if (disc < 0)
        return { inf, inf };

    float root = sqrt(disc);

    float t1 = (-b - root) / (2 * a);
    float t2 = (-b + root) / (2 * a);

    return { t1, t2 };
}

pair<Sphere*, float> closest_intersection(
    Vec3 O,
    Vec3 D,
    float t_min,
    float t_max,
    vector<Sphere>& spheres) {

    float closest_t = inf;
    Sphere* closest_sphere = nullptr;

    for (auto& sphere : spheres) {
        auto t = intersect_ray(O, D, sphere);

        if (t.first >= t_min && t.first <= t_max && t.first < closest_t) {
            closest_t = t.first;
            closest_sphere = &sphere;
        }

        if (t.second >= t_min && t.second <= t_max && t.second < closest_t) {
            closest_t = t.second;
            closest_sphere = &sphere;
        }
    }

    return { closest_sphere, closest_t };
}

float ComputeLighting(
    Vec3 P,
    Vec3 N,
    vector<Light>& lights,
    vector<Sphere>& spheres) {

    float intensity = 0.0f;

    for (auto& light : lights) {
        if (light.type == LIGHT_TYPE::ambient) {
            intensity += light.intensity;
            continue;
        }

        Vec3 L;
        float t_max;

        if (light.type == LIGHT_TYPE::point) {
            L = vec_sub(light.position, P);
            t_max = 1.0f;
        }
        else {
            L = light.direction;
            t_max = inf;
        }

        auto shadow = closest_intersection(P, L, 0.001f, t_max, spheres);

        if (shadow.first != nullptr)
            continue;

        float n_dot_l = dot(N, L);

        if (n_dot_l > 0) {
            intensity += light.intensity * n_dot_l /
                (length(N) * length(L));
        }
    }

    return intensity;
}

Color trace_ray(
    Vec3 O,
    Vec3 D,
    float t_min,
    float t_max,
    vector<Sphere>& spheres,
    vector<Light>& lights,
    int recursion_depth) {

    auto res = closest_intersection(O, D, t_min, t_max, spheres);

    Sphere* closest = res.first;
    float closest_t = res.second;

    if (closest == nullptr)
        return { 0, 0, 0 };

    Vec3 P = vec_add(O, vec_scale(D, closest_t));
    Vec3 N = normalize(vec_sub(P, closest->center));

    float light = ComputeLighting(P, N, lights, spheres);
    Color local_color = closest->color * light;

    float r = closest->reflective;

    if (recursion_depth <= 0 || r <= 0)
        return local_color;

    Vec3 reflected_ray = reflect_ray(-D, N);

    Color reflected_color = trace_ray(
        P,
        reflected_ray,
        0.001f,
        inf,
        spheres,
        lights,
        recursion_depth - 1);

    return local_color * (1.0f - r) + reflected_color * r;
}

float viewport_size = 1.0f;
float projection_plane_d = 1.0f;

Vec3 canvas_to_viewport(int x, int y) {
    return {
        x * viewport_size / width,
        y * viewport_size / height,
        projection_plane_d
    };
}

int main() {
    Vec3 camera = { 0, 0, 0 };

    vector<Sphere> spheres = {
        {{0, -1, 3}, 1, {255, 0, 0}, 0.2f},
        {{2, 0, 4}, 1, {0, 0, 255}, 0.4f},
        {{-2, 0, 4}, 1, {0, 255, 0}, 0.3f},
        {{0, -5001, 0}, 5000, {255, 255, 0}, 0.5f}
    };

    vector<Light> lights = {
        {LIGHT_TYPE::ambient, 0.2f, {0, 0, 0}, {0, 0, 0}},
        {LIGHT_TYPE::point, 0.6f, {2, 1, 0}, {0, 0, 0}},
        {LIGHT_TYPE::directional, 0.2f, {0, 0, 0}, {1, 4, 4}}
    };

    for (int x = -width / 2; x < width / 2; x++) {
        for (int y = -height / 2; y < height / 2; y++) {
            Vec3 D = normalize(canvas_to_viewport(x, y));

            Color c = trace_ray(
                camera,
                D,
                1,
                inf,
                spheres,
                lights,
                3);

            put_pixel(x, y, c);
        }
    }

    cout << "P3\n";
    cout << width << " " << height << "\n";
    cout << "255\n";

    for (auto c : image)
        cout << c.r << " " << c.g << " " << c.b << "\n";

    return 0;
}