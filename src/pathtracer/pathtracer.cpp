#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

    PathTracer::PathTracer() {
        gridSampler = new UniformGridSampler2D();
        hemisphereSampler = new UniformHemisphereSampler3D();

        tm_gamma = 2.2f;
        tm_level = 1.0f;
        tm_key = 0.18;
        tm_wht = 5.0f;
    }

    PathTracer::~PathTracer() {
        delete gridSampler;
        delete hemisphereSampler;
    }

    void PathTracer::set_frame_size(size_t width, size_t height) {
        sampleBuffer.resize(width, height);
        sampleCountBuffer.resize(width * height);
    }

    void PathTracer::clear() {
        bvh = NULL;
        grids = NULL;
        scene = NULL;
        camera = NULL;
        sampleBuffer.clear();
        sampleCountBuffer.clear();
        sampleBuffer.resize(0, 0);
        sampleCountBuffer.resize(0, 0);
    }

    void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                          size_t y0, size_t x1, size_t y1) {
        sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
    }

    Vector3D
    PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                    const Intersection &isect) {
        // Estimate the lighting from this intersection coming directly from a light.
        // For this function, sample uniformly in a hemisphere.

        // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
        // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

        // make a coordinate system for a hit point
        // with N aligned with the Z direction.
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        // w_out points towards the source of the ray (e.g.,
        // toward the camera if this is a primary ray)
        const Vector3D hit_p = r.o + r.d * isect.t;
        const Vector3D w_out = w2o * (-r.d);

        // This is the same number of total samples as
        // estimate_direct_lighting_importance (outside of delta lights). We keep the
        // same number of samples for clarity of comparison.
        int num_samples = scene->lights.size() * ns_area_light;
        Vector3D L_out;

        // TODO (Part 3): Write your sampling loop here
        // TODO BEFORE YOU BEGIN
        // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
        // get all lights
        Vector3D L_i = 0;
        for (int i = 0; i < num_samples; i++) {
            double pdf = 1.0 / (2.0 * PI);
            Vector3D w_in = hemisphereSampler->get_sample(); // w_in in object space, pointing away from the hit point

            const Vector3D wi = o2w * w_in; // get incoming in world space

            Ray target_ray = Ray(hit_p, wi);
            target_ray.min_t = EPS_F;
            Intersection intersection;
            if (bvh != NULL && bvh->intersect(target_ray, &intersection)) {
                Vector3D L = intersection.bsdf->get_emission();
                L_i += L * isect.bsdf->f(w_out, w_in) * cos_theta(w_in) / pdf;
            }
            else if (grids != NULL && grids->intersect(target_ray, &intersection)) {
                Vector3D L = intersection.bsdf->get_emission();
                L_i += L * isect.bsdf->f(w_out, w_in) * cos_theta(w_in) / pdf;
            }
        }
        L_out = L_i / num_samples;
        return L_out;
    }

    Vector3D
    PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                    const Intersection &isect) {
        // Estimate the lighting from this intersection coming directly from a light.
        // To implement importance sampling, sample only from lights, not uniformly in
        // a hemisphere.

        // make a coordinate system for a hit point
        // with N aligned with the Z direction.
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        // w_out points towards the source of the ray (e.g.,
        // toward the camera if this is a primary ray)
        const Vector3D hit_p = r.o + r.d * isect.t;
        const Vector3D w_out = w2o * (-r.d);
        Vector3D L_out;
        int num_samples;
        for (SceneLight *light : scene->lights) {
            double distToLight, pdf;
            if (light->is_delta_light()) {
                num_samples = 1;
            }
            else {
                num_samples = ns_area_light;
            }
            Vector3D L_in;
            Vector3D wi;
            for (int i = 0; i < num_samples; i++) {
                Vector3D v = light->sample_L(hit_p, &wi, &distToLight, &pdf);
                Vector3D w_in = w2o * wi;
                Ray target_ray = Ray(hit_p, wi);
                target_ray.min_t = EPS_F;
                target_ray.max_t = distToLight - EPS_F;

                if (w_in.z < 0) {
                    continue;
                }
                if (bvh != NULL && !(bvh->has_intersection(target_ray))){
                    L_in += isect.bsdf->f(w_out, w_in) * cos_theta(w_in) * v / pdf;
                }
                else if (grids != NULL && !(grids->has_intersection(target_ray))){
                    L_in += isect.bsdf->f(w_out, w_in) * cos_theta(w_in) * v / pdf;
                }
            }
            L_out += L_in / num_samples;
        }
        return L_out;
    }

    Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                              const Intersection &isect) {
        // TODO: Part 3, Task 2
        // Returns the light that results from no bounces of light
        return isect.bsdf->get_emission();
    }

    Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                             const Intersection &isect) {
        // TODO: Part 3, Task 3
        // Returns either the direct illumination by hemisphere or importance sampling
        // depending on `direct_hemisphere_sample`
        if (direct_hemisphere_sample) {
            return estimate_direct_lighting_hemisphere(r, isect);
        } else {
            return estimate_direct_lighting_importance(r, isect);
        }
    }

    Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                      const Intersection &isect) {
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        Vector3D hit_p = r.o + r.d * isect.t;
        Vector3D w_out = w2o * (-r.d);

        Vector3D L_out = one_bounce_radiance(r, isect);
//        Vector3D L_out;
        Vector3D w_in;
        double pdf;
        Vector3D v = isect.bsdf->sample_f(w_out, &w_in, &pdf);
        double terminationProb = 0.65;
        if (max_ray_depth == 0)
            return 0;
        if (max_ray_depth <= 1) {
            return L_out;
        } else if (coin_flip(terminationProb) || r.depth == max_ray_depth) {
            Vector3D direction = o2w * w_in; // get direction of current
            Ray bounce = Ray(hit_p + EPS_F * direction, direction, (int) r.depth - 1);
            Intersection intersection;
            // has next bounce
            if (bvh != NULL && bvh->intersect(bounce, &intersection)) {
                Vector3D next = at_least_one_bounce_radiance(bounce, intersection);
                L_out = L_out + next * cos_theta(w_in) * v / pdf / terminationProb;
            }
            else if (grids != NULL && grids->intersect(bounce, &intersection)) {
                Vector3D next = at_least_one_bounce_radiance(bounce, intersection);
                L_out = L_out + next * cos_theta(w_in) * v / pdf / terminationProb;
            }
        }
        return L_out;
    }

    Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
        Intersection isect;
        Vector3D L_out;

        // You will extend this in assignment 3-2.
        // If no intersection occurs, we simply return black.
        // This changes if you implement hemispherical lighting for extra credit.

        // The following line of code returns a debug color depending
        // on whether ray intersection with triangles or spheres has
        // been implemented.
        //
        // REMOVE THIS LINE when you are ready to begin Part 3.

        if (bvh != NULL && !bvh->intersect(r, &isect))
            return envLight ? envLight->sample_dir(r) : L_out;
        else if (grids != NULL && !grids->intersect(r, &isect))
            return envLight ? envLight->sample_dir(r) : L_out;

        L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

        // TODO (Part 3): Return the direct illumination.
//        L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);
        // TODO (Part 4): Accumulate the "direct" and "indirect"
        // parts of global illumination into L_out rather than just direct
        L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
        return L_out;
    }

    void PathTracer::raytrace_pixel(size_t x, size_t y) {
        // TODO (Part 1.2):
        // Make a loop that generates num_samples camera rays and traces them
        // through the scene. Return the average Vector3D.
        // You should call est_radiance_global_illumination in this function.

        // TODO (Part 5):
        // Modify your implementation to include adaptive sampling.
        // Use the command line parameters "samplesPerBatch" and "maxTolerance"

        int num_samples = ns_aa;          // total samples to evaluate
        Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

        Vector3D avg_color(0, 0, 0);
//        for (int i = 0; i < num_samples; ++i) {
//            Vector2D sample = gridSampler->get_sample(); // get a random uniform sample from [0, 1] * [0, 1]
//            Vector2D position = sample + origin;
//            Ray r = camera->generate_ray(position.x / sampleBuffer.w, position.y / sampleBuffer.h);
//            Vector3D color = est_radiance_global_illumination(r);
//            avg_color += color;
//        }
//        avg_color /= num_samples;
//
//        sampleBuffer.update_pixel(avg_color, x, y);
//        sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
        // part 5
        double s1 = 0, s2 = 0, illuminance = 0, mean = 0, variance = 0, convergence = 0;
        bool stop = false;
        int count = 0;
        for (int i = 0; i < num_samples; ++i) {
            Vector2D sample = gridSampler->get_sample(); // get a random uniform sample from [0, 1] * [0, 1]
            Vector2D position = sample + origin;
            Ray r = camera->generate_ray(position.x / sampleBuffer.w, position.y / sampleBuffer.h);
            r.depth = max_ray_depth;
            Vector3D color = est_radiance_global_illumination(r);
            avg_color += color;
            count = count + 1;

            illuminance = color.illum();
            s1 = s1 + illuminance;
            s2 = s2 + pow(illuminance, 2);
            // dividable
            if (count % (int) samplesPerBatch == 0) {
                mean = s1 / (double) count;
                variance = (1 / (double) (count - 1)) * (s2 - pow(s1, 2) / (double) count);
                convergence = 1.96 * sqrt(variance) / sqrt(count);
                if (convergence <= (maxTolerance * mean)) stop = true;
            }
            if (stop) break;
        }
        avg_color /= count;
        sampleBuffer.update_pixel(avg_color, x, y);
        sampleCountBuffer[x + y * sampleBuffer.w] = count;
    }

    void PathTracer::autofocus(Vector2D loc) {
        Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
        Intersection isect;

        if(bvh != NULL)
            bvh->intersect(r, &isect);
        else if(grids != NULL)
            grids->intersect(r, &isect);

        camera->focalDistance = isect.t;
    }

} // namespace CGL
