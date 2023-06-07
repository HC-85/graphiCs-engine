#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include "linalg.h"
#include "graphengine.h"

#define INFINITE 1e8
#define PI 3.1415926535

int main(){
    //////////////////////////////////////////////////////////////////////////////////
    /* GENERAL OBJECTS */
    Camera camera = {
        .position = {.x = 0, .y = -3, .z = .6}, 
        .direction = {.x = 0, .y = 1, .z = -.2},
        .fov = 30,
        .screen_distance = 1};
    camera.direction = vecNormalize(camera.direction);
    //////////////////////////////////////////////////////////////////////////////////
    /* MATERIALS */
    phMaterial mat1 = {.ambient_ref = .5, .diffuse_ref = .7, .specular_ref = .9, .shininess = .5};

    //////////////////////////////////////////////////////////////////////////////////
    /* ILLUMINATION OBJECTS */
    phSphere _sun = {
        .proto = {
            .center = {0, 0, 1e8},
            .radius = 1e7},
        .hit_function = &phongSphereHit,
        .tracing_function = NULL
    };

    LightSource sun = {
        .color = {255,255,255},
        .geometry = _sun,
    };

    phSphere _key_light = {
        .proto = {
            .center = {5, 10, 10},
            .radius = 1},
        .hit_function = &phongSphereHit,
        .tracing_function = NULL
    };

    LightSource key_light = {
        .color = {255,255,255},
        .geometry = _key_light,
    };

    phSphere _fill_light = {
        .proto = {
            .center = {5, -5, -3},
            .radius = 1},
        .hit_function = &phongSphereHit,
        .tracing_function = NULL
    };

    LightSource fill_light = {
        .color = {150,150,150},
        .geometry = _fill_light,
    };

    phSphere _back_light = {
        .proto = {
            .center = {0, -10 -2},
            .radius = 1},
        .hit_function = &phongSphereHit,
        .tracing_function = NULL
    };

    LightSource back_light = {
        .color = {100,100,100},
        .geometry = _back_light,
    };

    /* IS SKY FULLY REPLACED BY AMBIENT?
    phProtoPlane _sky = {
        .origin = {0, 0, INFINITY},
        .direction = {0, 0, -1},
    };

    Plane sky = {
    .properties = _sky,
    .hit_function = &skyHitV1,
    .tracing_function = &floorTracingV2
    };

    //////////////////////////////////////////////////////////////////////////////////
    /* SCENE ARRANGEMENT */    
    phSphere sphere = {
        .material = {
            .ambient_ref = .6,
            .color = {100, 50, 150},
            .diffuse_ref = .7,
            .shininess = 2,
            .specular_ref = .2
        },
        .proto = {
            .center = {0, 0, 0},
            .radius = .5},
        .hit_function = &phongSphereHit,
        .tracing_function = &phongSphereTracing
    };

    phPlane floorplane = {
        .material = {
            .ambient_ref = .35,
            .color = {100, 100, 150},
            .diffuse_ref = .4,
            .shininess = 2,
            .specular_ref = .3
        },
        .proto = {
            .origin = {0, 0, -2},
            .direction = {0, 0, 1}},
        .hit_function = &phongFloorHit,
        .tracing_function = &phongFloorTracing
    };

    // sphere = 0, plane = 1
    void* obj_arr[2];
    obj_arr[0] = &sphere;
    obj_arr[1] = &floorplane;

    int obj_types_arr[2];
    obj_types_arr[0] = 0;
    obj_types_arr[1] = 1;

    void* lighting_arr[3];
    lighting_arr[0] = &key_light;
    lighting_arr[1] = &fill_light;
    lighting_arr[2] = &back_light;
    
    int light_types_arr[3];
    light_types_arr[0] = 0;
    light_types_arr[1] = 0;
    light_types_arr[2] = 0;

    Scene scene = { .objects = &obj_arr, .obj_types = obj_types_arr, 
                    .lighting = &lighting_arr, .light_types = light_types_arr, 
                    .obj_size = 2, .light_size = 3};

    //////////////////////////////////////////////////////////////////////////////////
    int pixel_height = 640;
    int pixel_width = 640;
    float aspect_ratio = pixel_height/pixel_width;
    
    float virtual_width = 2*camera.screen_distance*tan(camera.fov/2 * PI/180);
    float virtual_height = aspect_ratio*virtual_width;

    Vec3 x_tmp = {1, 1, 0};
    Vec3 x_norm = grahamSchmidt(camera.direction, x_tmp, 1);
    Vec3 y_tmp = vecCross(camera.direction, x_norm);
    Vec3 y_norm = vecNormalize(y_tmp);

    float virtual_pixel_height = virtual_height/pixel_height;
    float virtual_pixel_width = virtual_width/pixel_width;
    Vec3 x_screen = vecScalarMult(x_norm, virtual_pixel_height);
    Vec3 y_screen = vecScalarMult(y_norm, virtual_pixel_width);

    Vec3 screen_center = vecScalarMult(camera.direction, camera.screen_distance);
    Vec3 white = {255, 255, 255};
    Vec3 black = {0, 0, 0};

    phLightRay current_ray;
    current_ray.ray.origin = camera.position;

    Vec3 x_scaled, y_scaled, tmp1, tmp2;

    unsigned char* image = (unsigned char*) malloc(pixel_width * pixel_height * sizeof(unsigned char) * 3);
    unsigned char* current_pixel_channel = image;

    phLightRay reflected_ray;
    float closest_dist;
    int closest_ind;
    float intersect_dist;
    int curr_obj_type;
    int curr_light_type; 
    int max_tracing_depth = 1;

    for (int y_pixel = (int) -pixel_height/2; y_pixel< (int) pixel_height/2; y_pixel++)
    {
            //if (y_pixel == 0)  continue;
        if (y_pixel < 0)    y_scaled = vecScalarMult(y_norm, (y_pixel+.5)*virtual_pixel_height);
        else                y_scaled = vecScalarMult(y_norm, (y_pixel-.5)*virtual_pixel_height);

        for (int x_pixel = (int) -pixel_width/2; x_pixel< (int) pixel_width/2; x_pixel++)
        {
            //if (x_pixel == 0)   continue;
            if (x_pixel < 0)    x_scaled = vecScalarMult(x_norm, (x_pixel+.5)*virtual_pixel_width);
            else                x_scaled = vecScalarMult(x_norm, (x_pixel-.5)*virtual_pixel_width);

            tmp1 = vecAdd(screen_center, x_scaled);
            tmp2 = vecAdd(tmp1, y_scaled);
            current_ray.ray.direction = vecNormalize(tmp2);
            current_ray.color = white;

            reflected_ray = current_ray;
            for (int curr_tracing_depth = 0; curr_tracing_depth < max_tracing_depth; curr_tracing_depth ++)
            {
                closest_dist = INFINITY;
                // FIND CLOSEST INTERSECTION
                for (int i = 0; i < scene.obj_size; i++)
                {
                    curr_obj_type = scene.obj_types[i];
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
                    if (intersect_dist<closest_dist) 
                    {
                        closest_dist = intersect_dist;
                        closest_ind = i;
                    }
                };
                // COMPUTE TRACING FUNCTION ONLY FOR CLOSEST
                if (closest_dist < INFINITY) // THE RAY HIT AN OBJECT
                {
                    if (scene.obj_types[closest_ind] == 0) 
                    {
                        phSphere curr_obj = *((phSphere*)(((uint64_t*)scene.objects)[closest_ind]));
                        reflected_ray = curr_obj.tracing_function(curr_obj.proto, curr_obj.material, reflected_ray, scene);
                    }
                    else if (scene.obj_types[closest_ind] == 1) 
                    {
                        phPlane curr_obj = *((phPlane*)(((uint64_t*)scene.objects)[closest_ind]));
                        reflected_ray = curr_obj.tracing_function(curr_obj.proto, curr_obj.material, reflected_ray, scene);
                    }
                }
                else
                {
                    reflected_ray.color = black;
                }
                /* else // CHECK IF IT HIT A LIGHT SOURCE
                {
                    for (int j = 0; j < scene.light_size; j++)
                    {
                        curr_light_type = scene.light_types[j];
                        if (curr_light_type == 0)
                        {
                            LightSource curr_light = *((LightSource*)(((uint64_t*)scene.lighting)[j]));
                            intersect_dist = curr_light.geometry.hit_function(curr_light.geometry.proto, reflected_ray);
                        }
                        else if (curr_light_type == 1)
                        {
                            LightSource curr_light = *((LightSource*)(((uint64_t*)scene.lighting)[j])); 
                            intersect_dist = curr_light.geometry.hit_function(curr_light.geometry.proto, reflected_ray);
                        }
                        if (intersect_dist<closest_dist) 
                        {
                            closest_dist = intersect_dist;
                            closest_ind = j;
                        }
                    }
                    curr_tracing_depth = max_tracing_depth;
                    if (closest_dist < INFINITY) // THE RAY HIT A LIGHTSOURCE 
                    {
                        if (scene.obj_types[closest_ind] == 0) 
                        {
                            Sphere curr_light = *((Sphere*)(((uint64_t*)scene.lighting)[closest_ind]));
                            reflected_ray = coloringV0(curr_light.properties.emission_color, curr_light.properties.reflectivity, reflected_ray);
                        }
                        else if (scene.light_types[closest_ind] == 1) 
                        {
                            Plane curr_light = *((Plane*)(((uint64_t*)scene.lighting)[closest_ind]));
                            reflected_ray = coloringV0(curr_light.properties.emission_color, curr_light.properties.reflectivity, reflected_ray);
                        }
                    }
                    else 
                }*/

            }
            *current_pixel_channel = reflected_ray.color.x; current_pixel_channel++;
            *current_pixel_channel = reflected_ray.color.y; current_pixel_channel++;
            *current_pixel_channel = reflected_ray.color.z; current_pixel_channel++;
        };
    };

    /////////////////////////// EXPORT IMAGE ////////////////////////////////
    array2ppm(image, "sphere.ppm", pixel_width, pixel_height, 255);
    return 0;
}