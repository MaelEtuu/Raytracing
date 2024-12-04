#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "material.h"

#include <thread>
#include <vector>
#include <functional>
#include <mutex>

std::vector<color> framebuffer;

class camera {
public:
	double aspect_ratio = 1;
	int    image_width = 100;
	int    samples_per_pixel = 10;
	int    max_depth = 10;

	double vfov = 90;
	point3 lookfrom = point3(0, 0, 0); // Point camera is looking from
	point3 lookat = point3(0, 0, -1); // Point camera is looking at
	vec3 vup = vec3(0, 1, 0); // Camera-relative "up" direction

	double defocus_angle = 0; // Variation angle of rays through each pixel
	double focus_dist = 10; // Distance from camera lookfrom point to plane of perfect focus


	/*void render(const hittable& world) {
		initialize();

		std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

		for (int j = 0; j < image_height; j++) {
			std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
			for (int i = 0; i < image_width; i++) {
				color pixel_color(0, 0, 0);
				for (int sample = 0; sample < samples_per_pixel; sample++) {
					ray r = get_ray(i, j);
					pixel_color += ray_color(r, max_depth, world);
				}
				write_color(std::cout, pixel_samples_scale * pixel_color);
			}
		}

		std::clog << "\rDone.                 \n";
	}*/

	std::mutex mutex;  // Pour éviter que plusieurs threads n'écrivent en même temps sur std::clog ou std::cout

	void render_section(int start, int end, const hittable& world) {
		for (int j = start; j < end; ++j) {
			std::clog << "\rScanlines remaining: " << end - j << ' ' << std::flush;
			for (int i = 0; i < image_width; ++i) {
				color pixel_color(0, 0, 0);
				for (int sample = 0; sample < samples_per_pixel; ++sample) {
					ray r = get_ray(i, j);
					pixel_color += ray_color(r, max_depth, world);
				}

				// Calcul de l'index dans le framebuffer
				int pixel_index = j * image_width + i;

				// Stockage du résultat dans le tampon
				framebuffer[pixel_index] = pixel_samples_scale * pixel_color;
			}
		}
	}

	void render_image_multithreaded(const hittable& world, int num_threads) {
		initialize();
		std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";
		framebuffer.resize(image_width * image_height); // Initialisation du framebuffer

		std::vector<std::thread> threads;
		int rows_per_thread = image_height / num_threads;

		for (int t = 0; t < num_threads; ++t) {
			int start_row = t * rows_per_thread;
			int end_row = (t == num_threads - 1) ? image_height : (t + 1) * rows_per_thread;

			threads.emplace_back([this, start_row, end_row, &world]() {
				this->render_section(start_row, end_row, world);
				});
		}

		// Joindre tous les threads
		for (auto& thread : threads) {
			thread.join();
		}

		// Après avoir calculé les couleurs, on écrit dans std::cout
		for (int j = 0; j < image_height; ++j) {
			for (int i = 0; i < image_width; ++i) {
				int pixel_index = j * image_width + i;
				write_color(std::cout, framebuffer[pixel_index]);
			}
		}

		std::clog << "\rDone.                 \n";
	}

private:
	int    image_height;  
	double pixel_samples_scale;
	point3 center;        
	point3 pixel00_loc;   
	vec3   pixel_delta_u; 
	vec3   pixel_delta_v;  
	vec3 u, v, w;
	vec3 defocus_disk_u; // Defocus disk horizontal radius
	vec3 defocus_disk_v; // Defocus disk vertical radius

	void initialize() {
		image_height = int(image_width / aspect_ratio);
		image_height = (image_height < 1) ? 1 : image_height;

		pixel_samples_scale = 1.0 / samples_per_pixel;

		center = lookfrom;

		//Camera
		auto theta = degrees_to_radians(vfov);
		auto h = std::tan(theta / 2);
		auto viewport_height = 2 * h * focus_dist;
		auto viewport_width = viewport_height * (double(image_width) / image_height);

		w = unit_vector(lookfrom - lookat);
		u = unit_vector(cross(vup, w));
		v = cross(w, u);

		//Vector x and y edge
		vec3 viewport_u = viewport_width * u;
		vec3 viewport_v = viewport_height * -v;

		//Calculate the bounds between section from pixel to pixel
		pixel_delta_u = viewport_u / image_width;
		pixel_delta_v = viewport_v / image_height;

		//Coord of the upper left pixel
		auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
		pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

		auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
		defocus_disk_u = u * defocus_radius;
		defocus_disk_v = v * defocus_radius;
	}

	ray get_ray(int i, int j) const {
		auto offset = sample_square();
		auto pixel_sample = pixel00_loc
			+ ((i + offset.x()) * pixel_delta_u)
			+ ((j + offset.y()) * pixel_delta_v);

		auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
		auto ray_direction = pixel_sample - ray_origin;
		auto ray_time = random_double();

		return ray(ray_origin, ray_direction, ray_time);
	}

	vec3 sample_square() const {
		return vec3(random_double() - 0.5, random_double() - 0.5, 0);
	}

	point3 defocus_disk_sample() const {
		auto p = random_in_unit_disk();
		return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
	}

	color ray_color(const ray& r, int depth, const hittable& world) const {
		if (depth <= 0) {
			return color(0, 0, 0);
		}

		hit_record rec;

		if (world.hit(r, interval(0.001, infinity), rec)) {
			ray scattered;
			color attenuation;
			if (rec.mat->scatter(r, rec, attenuation, scattered)) {
				return attenuation * ray_color(scattered, depth - 1, world);
			}
			return color(0, 0, 0);
		}

		vec3 unit_direction = unit_vector(r.direction());
		auto a = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
	}
};

#endif