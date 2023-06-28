#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include "linalg.h"
#include "graphengine.h"

#define PI 3.1415926535

int main(){
    //////////////////////////////////////////////////////////////////////////////////
    /* MATERIALS */
    SellmeierCoeffs BK7 = {.B1 = 1.03961212, .B2 = 0.231792344, .B3 = 1.01046945,.C1 = 6.00069867e-3, .C2 = 2.00179144e-2, .C3 = 103.560653};
    SellmeierCoeffs non_transparent = {0,0,0,0,0,0};
    //////////////////////////////////////////////////////////////////////////////////
    /* OBJECTS */
    /*    
    phSphere sphere = {
        .material = {
            .ambient_ref = .6,
            .diffuse_ref = .7,
            .specular_ref = .2,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {100, 50, 150}
        },
        .proto = {
            .center = {0, 0, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing2
    };


    phSphere sphere2 = {
        .material = {
            .ambient_ref = .6,
            .diffuse_ref = .7,
            .specular_ref = .2,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {150, 50, 100}
        },
        .proto = {
            .center = {1.25, 1.25, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing2
    };


    phSphere sphere3 = {
        .material = {
            .ambient_ref = .6,
            .diffuse_ref = .7,
            .specular_ref = .2,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {50, 150, 100}
        },
        .proto = {
            .center = {-1.25, -1.25, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing2
    };

    phPlane floorplane = {
        .material = {
            .ambient_ref = .35,
            .diffuse_ref = .4,
            .specular_ref = .3,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {100, 100, 150}
        },
        .proto = {
            .origin = {0, 0, -.5},
            .direction = {0, 0, 1}},
        .hit_function = &phongFloorHit,
        .tracing_function = &phongFloorTracing2
    };
    */
    frSphere sphere = {
        .material = {
            .ambient_ref = .6,
            .diffuse_ref = .7,
            .specular_ref = .2,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {100, 50, 150},
            .sellmeier_coeffs = BK7
        },
        .proto = {
            .center = {0, 0, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing2
    };


    frSphere sphere2 = {
        .material = {
            .ambient_ref = .6,
            .diffuse_ref = .7,
            .specular_ref = .2,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {150, 50, 100},
            .sellmeier_coeffs = BK7

        },
        .proto = {
            .center = {1.25, 1.25, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing2
    };


    frSphere sphere3 = {
        .material = {
            .ambient_ref = .6,
            .diffuse_ref = .7,
            .specular_ref = .2,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {50, 150, 100},
            .sellmeier_coeffs = BK7
        },
        .proto = {
            .center = {-1.25, -1.25, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing2
    };

    frPlane floorplane = {
        .material = {
            .ambient_ref = .35,
            .diffuse_ref = .4,
            .specular_ref = .3,
            .shininess = 2,
            .reflectiveness = .4,
            .color = {100, 100, 150},
            .sellmeier_coeffs = non_transparent
        },
        .proto = {
            .origin = {0, 0, -.5},
            .direction = {0, 0, 1}},
        .hit_function = &phongFloorHit,
        .tracing_function = &phongFloorTracing2
    };
    void* obj_arr[4];
    int obj_types_arr[4];
    // sphere = 0, plane = 1

    obj_arr[0] = &sphere;
    obj_types_arr[0] = 0;

    obj_arr[1] = &sphere2;
    obj_types_arr[1] = 0;
    
    obj_arr[2] = &sphere3;
    obj_types_arr[2] = 0;

    obj_arr[3] = &floorplane;
    obj_types_arr[3] = 1;

    //////////////////////////////////////////////////////////////////////////////////
    /* ILLUMINATION */
    PointLightSource key_light = {
        .color = {255,255,255},
        .position =  {5, 10, 10},
    };

    PointLightSource fill_light = {
        .color = {150,150,150},
        .position = {5, -5, -3},
    };

    PointLightSource back_light = {
        .color = {100,100,100},
        .position = {0, -10 -2},
    };

    void* lighting_arr[3];
    lighting_arr[0] = &key_light;
    lighting_arr[1] = &fill_light;
    lighting_arr[2] = &back_light;
    
    // pointsource = 0
    int light_types_arr[3];
    light_types_arr[0] = 0;
    light_types_arr[1] = 0;
    light_types_arr[2] = 0;

    //////////////////////////////////////////////////////////////////////////////////
    /* SCENE SETUP */
    Scene scene = { .objects = &obj_arr, .obj_types = obj_types_arr, 
                    .lighting = &lighting_arr, .light_types = light_types_arr, 
                    .obj_size = 4, .light_size = 3};

    Camera camera = {
        .position = {.x = -.5, .y = -4, .z = .6}, 
        .direction = {.x = 0, .y = 1, .z = -.2},
        .fov = 50,
        .screen_distance = 1}; camera.direction = vecNormalize(camera.direction);

    //////////////////////////////////////////////////////////////////////////////////
    /* IMAGE SETUP */
    int pixel_height = 256;
    int pixel_width = 256;
    unsigned char* image = (unsigned char*) malloc(pixel_width * pixel_height * sizeof(unsigned char) * 3);
    unsigned char* current_pixel_channel = image;

    //////////////////////////////////////////////////////////////////////////////////
    /* CANVAS SETUP */
    float aspect_ratio = pixel_height/pixel_width;
    
    float virtual_width = 2*camera.screen_distance*tan(camera.fov/2 * PI/180);
    float virtual_height = aspect_ratio*virtual_width;

    Vec3 x_norm = grahamSchmidt(camera.direction, (Vec3) {1, 1, 0}, 1);
    Vec3 y_norm = vecNormalize(vecCross(camera.direction, x_norm));

    float virtual_pixel_height = virtual_height/pixel_height;
    float virtual_pixel_width = virtual_width/pixel_width;
    Vec3 x_screen = vecScalarMult(x_norm, virtual_pixel_height);
    Vec3 y_screen = vecScalarMult(y_norm, virtual_pixel_width);

    Vec3 screen_center = vecScalarMult(camera.direction, camera.screen_distance);

    //////////////////////////////////////////////////////////////////////////////////
    /* RAY SETUP */
    frLightRay current_ray;
    // frLightRay reflected_ray;
    int max_tracing_depth = 100;
    for (int y_pixel = (int) -pixel_height/2; y_pixel< (int) pixel_height/2; y_pixel++)
    {
        //if (y_pixel == 0)  continue;
        Vec3 y_scaled;
        if (y_pixel < 0)    y_scaled = vecScalarMult(y_norm, (y_pixel+.5)*virtual_pixel_height);
        else                y_scaled = vecScalarMult(y_norm, (y_pixel-.5)*virtual_pixel_height);

        for (int x_pixel = (int) -pixel_width/2; x_pixel< (int) pixel_width/2; x_pixel++)
        {
            //if (x_pixel == 0)   continue;
            Vec3 x_scaled;
            if (x_pixel < 0)    x_scaled = vecScalarMult(x_norm, (x_pixel+.5)*virtual_pixel_width);
            else                x_scaled = vecScalarMult(x_norm, (x_pixel-.5)*virtual_pixel_width);

            current_ray.ray.origin = camera.position;
            current_ray.ray.direction = vecNormalize(vecAdd(vecAdd(screen_center, x_scaled), y_scaled));
            //current_ray.color = (Vec3) {0, 0, 0};
            current_ray.wavelength = RED_WAVELENGTH;
            
            int test = fresnelTracing(current_ray, scene, x_pixel, y_pixel);
            if (test == 42) return 0;
            // fresnelBackwardColoring
            /*
            reflected_ray = current_ray;
            for (int curr_tracing_depth = 1; curr_tracing_depth <= max_tracing_depth; curr_tracing_depth ++)
            {
                float closest_dist = INFINITY;
                float intersect_dist;
                int closest_ind;

                // FIND CLOSEST INTERSECTION
                for (int i = 0; i < scene.obj_size; i++)
                {
                    int curr_obj_type = scene.obj_types[i];
                    if (curr_obj_type == 0) // Sphere
                    {
                        phSphere curr_obj = *((phSphere*)(((uint64_t*)scene.objects)[i]));
                        intersect_dist = curr_obj.hit_function(curr_obj.proto, reflected_ray);
                    }
                    else if (curr_obj_type == 1) // Plane
                    {
                        phPlane curr_obj = *((phPlane*)(((uint64_t*)scene.objects)[i])); 
                        intersect_dist = curr_obj.hit_function(curr_obj.proto, reflected_ray);
                    }

                    if (intersect_dist<closest_dist) // SAVE CLOSEST INTERSECTION
                    {
                        closest_dist = intersect_dist;
                        closest_ind = i;
                    }
                };
                // COMPUTE TRACING FUNCTION ONLY FOR CLOSEST
                if (closest_dist < INFINITY) // IF THE RAY HIT AN OBJECT
                {
                    if (scene.obj_types[closest_ind] == 0) // THE OBJECT WAS A SPHERE
                    {  
                        phSphere curr_obj = *((phSphere*)(((uint64_t*)scene.objects)[closest_ind]));
                        reflected_ray = curr_obj.tracing_function(curr_obj.proto, curr_obj.material, reflected_ray, scene, curr_tracing_depth);
                    }
                    else if (scene.obj_types[closest_ind] == 1) // THE OBJECT WAS A PLANE
                    {
                        phPlane curr_obj = *((phPlane*)(((uint64_t*)scene.objects)[closest_ind]));
                        reflected_ray = curr_obj.tracing_function(curr_obj.proto, curr_obj.material, reflected_ray, scene, curr_tracing_depth);
                    }
                }
                // THE RAY DID NOT HIT AN OBJECT
                else 
                {
                    if (curr_tracing_depth == 1) reflected_ray.color = (Vec3) {0, 0, 0};
                    else break;
                }
            }
            // COLOR THE IMAGE ARRAY
            *current_pixel_channel = reflected_ray.color.x; current_pixel_channel++;
            *current_pixel_channel = reflected_ray.color.y; current_pixel_channel++;
            *current_pixel_channel = reflected_ray.color.z; current_pixel_channel++;
            */
        };
    };
    /*
    /////////////////////////// EXPORT IMAGE ////////////////////////////////
    array2ppm(image, "sphere.ppm", pixel_width, pixel_height, 255);
    return 0;
    */
}