#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

using namespace std;

const int width = 800;
const int height = 600;
const float inf = numeric_limits<float>::infinity();

enum class LIGHT_TYPE { ambient, directional, point };

struct Vec3 {
    float x, y, z;
};

struct Color {
    int r, g, b;
};

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

Color operator*(Color c, float intensity) {
    return {
        clamp(static_cast<int>(c.r * intensity), 0, 255),
        clamp(static_cast<int>(c.g * intensity), 0, 255),
        clamp(static_cast<int>(c.b * intensity), 0, 255)
    };
}

Color operator+(Color a, Color b) {
    return {
        clamp(a.r + b.r, 0, 255),
        clamp(a.g + b.g, 0, 255),
        clamp(a.b + b.b, 0, 255)
    };
}

Vec3 operator-(Vec3 v) {
    return { -v.x, -v.y, -v.z };
}

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
    if (l == 0.0f)
        return { 0.0f, 0.0f, 0.0f };

    return { v.x / l, v.y / l, v.z / l };
}

Vec3 reflect_ray(Vec3 R, Vec3 N) {
    return vec_sub(vec_scale(N, 2.0f * dot(N, R)), R);
}

// Rotate vector using Yaw (Y-axis) and Pitch (X-axis) in Radians
Vec3 rotate_vector(Vec3 v, float yaw_deg, float pitch_deg) {
    float yaw_rad = yaw_deg * 3.14159265f / 180.0f;
    float pitch_rad = pitch_deg * 3.14159265f / 180.0f;

    // Pitch rotation around X axis
    float y1 = v.y * cos(pitch_rad) - v.z * sin(pitch_rad);
    float z1 = v.y * sin(pitch_rad) + v.z * cos(pitch_rad);
    float x1 = v.x;

    // Yaw rotation around Y axis
    float x2 = x1 * cos(yaw_rad) + z1 * sin(yaw_rad);
    float z2 = -x1 * sin(yaw_rad) + z1 * cos(yaw_rad);
    float y2 = y1;

    return { x2, y2, z2 };
}

void put_pixel(vector<Color>& res, int x, int y, Color color) {
    int px = x + width / 2;
    int py = height / 2 - y - 1;

    if (px < 0 || px >= width || py < 0 || py >= height)
        return;

    res[py * width + px] = color;
}

pair<float, float> intersect_ray(Vec3 O, Vec3 D, const Sphere& s) {
    Vec3 CO = vec_sub(O, s.center);
    float a = dot(D, D);
    float b = 2.0f * dot(CO, D);
    float c = dot(CO, CO) - s.radius * s.radius;
    float disc = b * b - 4.0f * a * c;

    if (disc < 0.0f)
        return { inf, inf };

    float root = sqrt(disc);
    float t1 = (-b - root) / (2.0f * a);
    float t2 = (-b + root) / (2.0f * a);

    return { t1, t2 };
}

pair<Sphere*, float> closest_intersection(Vec3 O, Vec3 D, float t_min, float t_max, vector<Sphere>& spheres) {
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

float ComputeLighting(Vec3 P, Vec3 N, vector<Light>& lights, vector<Sphere>& spheres) {
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
            float distance_to_light = length(L);

            if (distance_to_light == 0.0f)
                continue;

            L = normalize(L);
            t_max = distance_to_light;
        }
        else {
            L = normalize(light.direction);
            t_max = inf;
        }

        auto shadow = closest_intersection(P, L, 0.001f, t_max, spheres);

        if (shadow.first != nullptr)
            continue;

        float n_dot_l = dot(N, L);

        if (n_dot_l > 0.0f)
            intensity += light.intensity * n_dot_l;
    }

    return min(intensity, 1.0f);
}

Color trace_ray(Vec3 O, Vec3 D, float t_min, float t_max, vector<Sphere>& spheres, vector<Light>& lights, int recursion_depth) {
    auto res = closest_intersection(O, D, t_min, t_max, spheres);
    Sphere* closest = res.first;
    float closest_t = res.second;

    if (closest == nullptr)
        return { 0, 0, 0 };

    Vec3 P = vec_add(O, vec_scale(D, closest_t));
    Vec3 N = normalize(vec_sub(P, closest->center));
    float light = ComputeLighting(P, N, lights, spheres);
    Color local_color = closest->color * light;
    float r = clamp(closest->reflective, 0.0f, 1.0f);

    if (recursion_depth <= 0 || r <= 0.0f)
        return local_color;

    Vec3 reflected_ray = reflect_ray(-D, N);

    Color reflected_color = trace_ray(P, reflected_ray, 0.001f, inf, spheres, lights, recursion_depth - 1);

    return local_color * (1.0f - r) + reflected_color * r;
}

float viewport_size = 1.0f;
float projection_plane_d = 1.0f;

Vec3 canvas_to_viewport(int x, int y) {
    float viewport_width = viewport_size * static_cast<float>(width) / static_cast<float>(height);

    return {
        x * viewport_width / width,
        y * viewport_size / height,
        projection_plane_d
    };
}

void get_curr_frame(vector<Color>& res, Vec3& camera, float yaw, float pitch) {
    vector<Sphere> spheres = {
        {{0, -1, 3}, 1, {255, 0, 0}, 0.2f},
        {{2, 0, 4}, 1, {0, 0, 255}, 0.4f},
        {{-2, 0, 4}, 1, {0, 255, 0}, 0.3f},
        {{0, -5001, 0}, 5000, {255, 255, 0}, 0.5f}
    };

    vector<Light> lights = {
        { LIGHT_TYPE::ambient, 0.2f, {0, 0, 0}, {0, 0, 0} },
        { LIGHT_TYPE::point, 0.6f, {2, 1, 0}, {0, 0, 0} },
        { LIGHT_TYPE::directional, 0.2f, {0, 0, 0}, {1, 4, 4} }
    };

    for (int x = -width / 2; x < width / 2; x++) {
        for (int y = -height / 2; y < height / 2; y++) {
            Vec3 D = normalize(canvas_to_viewport(x, y));
            // Apply camera rotation matrix/transformation to ray direction
            D = rotate_vector(D, yaw, pitch);

            Color c = trace_ray(camera, D, 1.0f, inf, spheres, lights, 3);
            put_pixel(res, x, y, c);
        }
    }
}

void fill_texture(SDL_Texture* texture, vector<Color>& image) {
    void* pixels = nullptr;
    int pitch = 0;

    // FIX: In SDL3, SDL_LockTexture returns true on success, false on failure
    if (!SDL_LockTexture(texture, nullptr, &pixels, &pitch)) {
        cerr << "Failed to lock texture: " << SDL_GetError() << '\n';
        return;
    }

    auto* pixel_buffer = static_cast<Uint8*>(pixels);

    for (int y = 0; y < height; y++) {
        auto* row = reinterpret_cast<Uint32*>(pixel_buffer + y * pitch);

        for (int x = 0; x < width; x++) {
            const Color& color = image[y * width + x];
            row[x] = (static_cast<Uint32>(color.r) << 24) |
                (static_cast<Uint32>(color.g) << 16) |
                (static_cast<Uint32>(color.b) << 8) |
                255;
        }
    }

    SDL_UnlockTexture(texture);
}

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Ray Tracer", width, height, SDL_WINDOW_RESIZABLE);

    if (!window) {
        cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    if (!renderer) {
        cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, width, height);

    if (!texture) {
        cerr << "SDL_CreateTexture failed: " << SDL_GetError() << '\n';
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    vector<Color> image(width * height);
    Vec3 camera = { 0.0f, 0.0f, 0.0f };
    bool running = true;
    bool needs_render = true;
    const float speed = 3.0f;
    auto previous_time = chrono::steady_clock::now();

    float yaw = 0.0f;
    float pitch = 0.0f;
    float sensitivity = 0.1f;
    SDL_SetWindowRelativeMouseMode(window, true);

    while (running) {
        auto current_time = chrono::steady_clock::now();
        float delta_time = chrono::duration<float>(current_time - previous_time).count();
        previous_time = current_time;

        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT)
                running = false;

            if (event.type == SDL_EVENT_MOUSE_MOTION) {
                float dx = (float)event.motion.xrel;
                float dy = (float)event.motion.yrel;

                yaw += dx * sensitivity;
                pitch += dy * sensitivity;
                if (pitch > 89.0f)  pitch = 89.0f;
                if (pitch < -89.0f) pitch = -89.0f;

                needs_render = true;
            }
        }

        const bool* keyboard = SDL_GetKeyboardState(nullptr);
        float movement = speed * delta_time;

        // FIX: Calculate camera forward/right directions based on yaw so WASD moves relative to where you look
        Vec3 forward = rotate_vector({ 0.0f, 0.0f, 1.0f }, yaw, 0.0f);
        Vec3 right = rotate_vector({ 1.0f, 0.0f, 0.0f }, yaw, 0.0f);

        if (keyboard[SDL_SCANCODE_W]) {
            camera = vec_add(camera, vec_scale(forward, movement));
            needs_render = true;
        }
        if (keyboard[SDL_SCANCODE_S]) {
            camera = vec_sub(camera, vec_scale(forward, movement));
            needs_render = true;
        }
        if (keyboard[SDL_SCANCODE_A]) {
            camera = vec_sub(camera, vec_scale(right, movement));
            needs_render = true;
        }
        if (keyboard[SDL_SCANCODE_D]) {
            camera = vec_add(camera, vec_scale(right, movement));
            needs_render = true;
        }

        if (needs_render) {
            get_curr_frame(image, camera, yaw, pitch);
            fill_texture(texture, image);
            needs_render = false;
        }

        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}