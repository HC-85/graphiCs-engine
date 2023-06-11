#ifndef graphengine
#define graphengine
#define PI 3.1415926535
#include "linalg.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
/////////////////////////////////////////////////////////////////////////////
/* PROTO OBJECTS */
typedef struct
{
    float radius;
    float transparency;
    float reflectivity;
    Vec3 center;
    Vec3 surface_color;
    Vec3 emission_color;
} protoSphere;

typedef struct
{
    Vec3 origin;
    Vec3 direction;
    Vec3 surface_color;
    Vec3 emission_color;
    float reflectivity;
    float transparency;
} protoPlane;

typedef struct
{
    float radius;
    Vec3 center;
} phProtoSphere;

typedef struct
{
    Vec3 origin;
    Vec3 direction;
} phProtoPlane;

/////////////////////////////////////////////////////////////////////////////
/* BASIC RENDERING OBJECTS */
typedef struct
{
    Vec3 position;
    Vec3 direction;
    float fov;
    float screen_distance;
} Camera;

typedef struct
{
    Vec3 origin;
    Vec3 direction;
} Ray;

typedef struct
{
    Vec3 origin;
    Vec3 direction;
    Vec3 color; 
} LightRay;

typedef struct
{
    Ray ray;
    Vec3 color; 
} phLightRay;

typedef struct
{
    float specular_ref;
    float diffuse_ref;
    float ambient_ref;
    float shininess; 
    Vec3 color;
} phMaterial;

typedef struct
{
    void* objects;
    void* lighting;
    int* obj_types;
    int* light_types;
    int obj_size;
    int light_size; 
} Scene;

/////////////////////////////////////////////////////////////////////////////
/* SCENE OBJECTS */
typedef struct
{
    protoSphere properties;
    float (*hit_function)(protoSphere, LightRay);
    LightRay (*tracing_function)(protoSphere, LightRay);
} Sphere;

typedef struct
{
    protoPlane properties;
    float (*hit_function)(phProtoPlane, LightRay);
    LightRay (*tracing_function)(phProtoPlane, LightRay);
} Plane;

typedef struct
{
    phProtoSphere proto;
    phMaterial material;
    float (*hit_function)(phProtoSphere, phLightRay);
    phLightRay (*tracing_function)(phProtoSphere, phMaterial, phLightRay, Scene);
} phSphere;

typedef struct
{
    phProtoPlane proto;
    phMaterial material;
    float (*hit_function)(phProtoPlane, phLightRay);
    phLightRay (*tracing_function)(phProtoPlane, phMaterial, phLightRay, Scene);
} phPlane;

/////////////////////////////////////////////////////////////////////////////
/* LIGHTING */
typedef union {
    phSphere sphere;
    phPlane plane;
} Geometry;

typedef union {
    float (*hit_function_sphere)(phProtoSphere, phLightRay);
    float (*hit_function_plane)(phProtoPlane, phLightRay);
} HitFunction;

typedef struct
{
    Vec3 color;
    Geometry geometry;
} LightSource;

typedef struct
{
    Vec3 color;
    Vec3 position;
} PointLightSource;

/////////////////////////////////////////////////////////////////////////////
/* HIT FUNCTIONS */
float sphereHit(protoSphere sphere, LightRay lightray)
{
    Vec3 neg_lightray_origin = vecScalarMult((lightray.origin), -1);
    Vec3 cam_to_sphere_center = vecAdd((sphere.center), neg_lightray_origin);
    float adjacent = vecDot(cam_to_sphere_center, (lightray.direction));
    int is_behind  = (adjacent < 0);
    if (is_behind) return INFINITY;

    float hyp2 = vecDot(cam_to_sphere_center, cam_to_sphere_center);
    float op2 = hyp2 - adjacent*adjacent;
    int is_miss = (op2 > (sphere.radius * sphere.radius));
    if (is_miss) return INFINITY;

    Vec3 projection = vecScalarMult(lightray.direction, adjacent);
    Vec3 neg_projection = vecScalarMult(projection, -1);
    Vec3 perp_vec = vecAdd(cam_to_sphere_center, neg_projection);
    
    float proj_surf_dist = sqrt(sphere.radius*sphere.radius - vecDot(perp_vec, perp_vec));    
    float intersect_distance = adjacent - proj_surf_dist;
    return intersect_distance;
}

float phongSphereHit(phProtoSphere sphere, phLightRay lightray)
{
    Vec3 neg_lightray_origin = vecScalarMult((lightray.ray.origin), -1);
    Vec3 cam_to_sphere_center = vecAdd((sphere.center), neg_lightray_origin);
    float adjacent = vecDot(cam_to_sphere_center, (lightray.ray.direction));
    int is_behind  = (adjacent < 0);
    if (is_behind) return INFINITY;

    float hyp2 = vecDot(cam_to_sphere_center, cam_to_sphere_center);
    float op2 = hyp2 - adjacent*adjacent;
    int is_miss = (op2 > (sphere.radius * sphere.radius));
    if (is_miss) return INFINITY;

    Vec3 projection = vecScalarMult(lightray.ray.direction, adjacent);
    Vec3 neg_projection = vecScalarMult(projection, -1);
    Vec3 perp_vec = vecAdd(cam_to_sphere_center, neg_projection);
    
    float proj_surf_dist = sqrt(sphere.radius*sphere.radius - vecDot(perp_vec, perp_vec));    
    float intersect_distance = adjacent - proj_surf_dist;
    return intersect_distance;
}


float floorHit(protoPlane floorplane, LightRay lightray)
{
    int is_miss = (lightray.direction.z>=0);
    if (is_miss) return INFINITY;

    float distance_to_floor = lightray.origin.z + ABS(floorplane.origin.z);
    float dist_over_zdir = distance_to_floor/ABS(lightray.direction.z);
    float x_intersect = lightray.origin.x + lightray.direction.x*dist_over_zdir;
    float y_intersect = lightray.origin.y + lightray.direction.y*dist_over_zdir;
    Vec3 intersect = {x_intersect, y_intersect, floorplane.origin.z};

    Vec3 tmp = vecScalarMult(lightray.origin, -1);
    Vec3 cam_to_intersect = vecAdd(intersect, tmp);
    float intersect_distance = vecMagnitude(cam_to_intersect);
    
    return intersect_distance; 
}

float phongFloorHit(phProtoPlane floorplane, phLightRay lightray)
{
    int is_miss = (lightray.ray.direction.z>=0);
    if (is_miss) return INFINITY;

    float distance_to_floor = lightray.ray.origin.z + ABS(floorplane.origin.z);
    float dist_over_zdir = distance_to_floor/ABS(lightray.ray.direction.z);
    float x_intersect = lightray.ray.origin.x + lightray.ray.direction.x*dist_over_zdir;
    float y_intersect = lightray.ray.origin.y + lightray.ray.direction.y*dist_over_zdir;
    Vec3 intersect = {x_intersect, y_intersect, floorplane.origin.z};

    Vec3 tmp = vecScalarMult(lightray.ray.origin, -1);
    Vec3 cam_to_intersect = vecAdd(intersect, tmp);
    float intersect_distance = vecMagnitude(cam_to_intersect);
    
    return intersect_distance; 
}


float skyHit(protoPlane floorplane, LightRay lightray)
{
    int is_miss = (lightray.direction.z<0);
    if (is_miss) return INFINITY;
    return 0; 
}


/////////////////////////////////////////////////////////////////////////////
/* TRACING FUNCTIONS */
Vec3 phongIllumination(Vec3 ambient_color, phMaterial material, Scene scene, Vec3 intersection, Vec3 surf2cam_direction, Vec3 surf_normal, Vec3 reflected_direction)
{
    assert(ABS(vecMagnitude(surf2cam_direction)-1)<1e-3);
    assert(ABS(vecMagnitude(surf_normal)-1)<1e-3);
    assert(ABS(vecMagnitude(reflected_direction)-1)<1e-3);

    // Ambient light
    Vec3 total_illum = vecScalarMult(ambient_color, material.ambient_ref);
    // Should I declare it as static inside the loop? Use curly braces inside the loop to limit its scope but keep it through iterations?

    for (int j = 0; j < scene.light_size; j++)
    {
        
        int curr_light_type = scene.light_types[j];
        if (curr_light_type == 0) // Point Source
        {
            PointLightSource curr_light = *((PointLightSource*)(((uint64_t*)scene.lighting)[j]));
            Vec3 source_dir = vecAdd(curr_light.position, vecScalarMult(intersection, -1));
            source_dir = vecNormalize(source_dir);
            
            float proj_source_normal = vecDot(source_dir, surf_normal);
            if (proj_source_normal < 0) continue;

            Vec3 diff_color = vecScalarMult(material.color, material.diffuse_ref/255);
            Vec3 tmp = vec3ComponentMult(diff_color, curr_light.color); //WILL ASSUME SAME LIGHT INTENSITY FOR DIFF, SPEC AND AMB
            Vec3 diff_illum = vecScalarMult(tmp, proj_source_normal);
            total_illum = vecAdd(total_illum, diff_illum); //DIFFUSE TERM

            float proj_cam_ref = vecDot(surf2cam_direction, reflected_direction);
            if (proj_cam_ref < 0) continue;
            Vec3 spec_color = vecScalarMult(material.color, material.specular_ref/255);
            tmp = vec3ComponentMult(spec_color, curr_light.color);
            Vec3 spec_illum = vecScalarMult(tmp, powf(proj_cam_ref, material.shininess));
            total_illum = vecAdd(total_illum, spec_illum); //SPECULAR TERM

        } else if (curr_light_type == 1)
        {
            assert(0);
        }
    };
    // Vec3 scaled_total_illum = vecScalarMult(total_illum, 255); Intensities are already in [0,255]
    Vec3 clamped_total_illum = vec3Clamp(total_illum, 0, 255);
    return clamped_total_illum;
}


LightRay sphereTracing(protoSphere sphere, LightRay lightray)
{
    Vec3 neg_lightray_origin = vecScalarMult((lightray.origin), -1);
    Vec3 cam_to_sphere_center = vecAdd((sphere.center), neg_lightray_origin);
    float adjacent = vecDot(cam_to_sphere_center, (lightray.direction));

    float hyp2 = vecDot(cam_to_sphere_center, cam_to_sphere_center);
    float op2 = hyp2 - adjacent*adjacent;

    Vec3 projection = vecScalarMult(lightray.direction, adjacent);
    Vec3 neg_projection = vecScalarMult(projection, -1);
    Vec3 perp_vec = vecAdd(cam_to_sphere_center, neg_projection);
    
    float proj_surf_dist = sqrt(sphere.radius*sphere.radius - vecDot(perp_vec, perp_vec));    
    Vec3 surf = vecScalarMult(lightray.direction, adjacent - proj_surf_dist);
    
    Vec3 neg_cam_to_sphere_center = vecScalarMult(cam_to_sphere_center, -1);
    Vec3 surf_normal = vecAdd(surf, neg_cam_to_sphere_center);
    surf_normal = vecNormalize(surf_normal);
    Vec3 flip_vector = vecScalarMult(surf_normal, -2*vecDot(lightray.direction, surf_normal));
    Vec3 reflected_direction = vecAdd(lightray.direction, flip_vector);

    Vec3 reflectivity_unit = vecScalarMult(sphere.surface_color, sphere.reflectivity/255);
    Vec3 reflected_color = vec3ComponentMult(reflectivity_unit, lightray.color);

    LightRay reflected_lightray = {.origin = vecAdd(surf, lightray.origin), .direction = reflected_direction, .color = reflected_color};
    return reflected_lightray;
};


phLightRay phongSphereTracing(phProtoSphere sphere, phMaterial material, phLightRay lightray, Scene scene){
    Vec3 cam_to_sphere_center = vecAdd(sphere.center, vecScalarMult((lightray.ray.origin), -1));
    float adjacent = vecDot(cam_to_sphere_center, (lightray.ray.direction));

    float hyp2 = vecDot(cam_to_sphere_center, cam_to_sphere_center);
    float op2 = hyp2 - adjacent*adjacent;

    Vec3 projection = vecScalarMult(lightray.ray.direction, adjacent);
    Vec3 neg_projection = vecScalarMult(projection, -1);
    Vec3 perp_vec = vecAdd(cam_to_sphere_center, neg_projection);
    
    float proj_surf_dist = sqrt(sphere.radius*sphere.radius - vecDot(perp_vec, perp_vec));    
    Vec3 cam2surf = vecScalarMult(lightray.ray.direction, adjacent - proj_surf_dist);
    Vec3 intersection = vecAdd(cam2surf, lightray.ray.origin);

    Vec3 surf_normal = vecAdd(cam2surf,  vecScalarMult(cam_to_sphere_center, -1));
    surf_normal = vecNormalize(surf_normal);
    Vec3 flip_vector = vecScalarMult(surf_normal, -2*vecDot(lightray.ray.direction, surf_normal));
    Vec3 reflected_direction = vecAdd(lightray.ray.direction, flip_vector);
    reflected_direction = vecNormalize(reflected_direction);

    Vec3 white = {255,255,255};
    Vec3 surf2cam_direction = vecScalarMult(vecNormalize(cam2surf), -1);
    Vec3 illum = phongIllumination(white, material, scene, intersection, surf2cam_direction, surf_normal, reflected_direction);

    phLightRay reflected_lightray = {.ray = {.origin = intersection, .direction = reflected_direction}, .color = illum};
    return reflected_lightray;
};


LightRay floorTracing(protoPlane floorplane, LightRay lightray)
{
    float distance_to_floor = lightray.origin.z + ABS(floorplane.origin.z);
    float dist_over_zdir = distance_to_floor/ABS(lightray.direction.z);
    float x_intersect = lightray.origin.x + lightray.direction.x*dist_over_zdir;
    float y_intersect = lightray.origin.y + lightray.direction.y*dist_over_zdir;
    Vec3 intersect = {x_intersect, y_intersect, floorplane.origin.z};
    
    Vec3 flip_vector = vecScalarMult(floorplane.direction, -2*vecDot(lightray.direction, floorplane.direction));
    Vec3 reflected_direction = vecAdd(lightray.direction, flip_vector);

    Vec3 reflectivity_unit = vecScalarMult(floorplane.surface_color, floorplane.reflectivity/255);
    Vec3 reflected_color = vec3ComponentMult(reflectivity_unit, lightray.color);

    LightRay reflected_lightray = {.origin = intersect, .direction = reflected_direction, .color = reflected_color};
    return reflected_lightray;
};


phLightRay phongFloorTracing(phProtoPlane floorplane, phMaterial material, phLightRay lightray, Scene scene)
{
    float distance_to_floor = lightray.ray.origin.z + ABS(floorplane.origin.z);
    float dist_over_zdir = distance_to_floor/ABS(lightray.ray.direction.z);
    float x_intersect = lightray.ray.origin.x + lightray.ray.direction.x*dist_over_zdir;
    float y_intersect = lightray.ray.origin.y + lightray.ray.direction.y*dist_over_zdir;
    Vec3 intersection = {x_intersect, y_intersect, floorplane.origin.z};
    
    Vec3 flip_vector = vecScalarMult(floorplane.direction, -2*vecDot(lightray.ray.direction, floorplane.direction));
    Vec3 reflected_direction = vecAdd(lightray.ray.direction, flip_vector);

    Vec3 cam2surf = vecAdd(intersection, vecScalarMult(lightray.ray.origin, -1));

    Vec3 white = {255,255,255};
    Vec3 illum = phongIllumination(white, material, scene, intersection, vecNormalize(cam2surf), floorplane.direction, reflected_direction);

    phLightRay reflected_lightray = {.ray = {.origin = intersection, .direction = reflected_direction, }, .color = illum};
    return reflected_lightray;
};

/////////////////////////////////////////////////////////////////////////////
/* LIGHT SOURCE COLORING FUNCTIONS */

LightRay skyColoring(protoPlane sky, LightRay lightray)
{
    Vec3 reflectivity_unit = vecScalarMult(sky.surface_color, sky.reflectivity/255);
    Vec3 reflected_color = vec3ComponentMult(reflectivity_unit, lightray.color);

    LightRay reflected_lightray = {.origin = lightray.origin, .direction = lightray.direction, .color = reflected_color};
    return reflected_lightray;
}


LightRay coloring(Vec3 color, LightRay lightray, int curr_tracing_depth)
{
    Vec3 coloring_unit = vecScalarMult(color, 255);
    Vec3 reflected_color = vec3ComponentMult(coloring_unit, lightray.color);

    LightRay reflected_lightray = {.origin = lightray.origin, .direction = lightray.direction, .color = reflected_color};
    return reflected_lightray;
}

/////////////////////////////////////////////////////////////////////////////
/* TRACING MANAGER */
void tracingManagerV0(Camera camera, int pixel_height, int pixel_width, Scene scene, int max_recursion)
{
    unsigned char* image = (unsigned char*) malloc(pixel_width * pixel_height * sizeof(unsigned char) * 3);
    unsigned char* current_pixel_channel = image;

    Vec3 white = {255, 255, 255};
    Vec3 black = {0, 0, 0};

    float aspect_ratio = pixel_height/pixel_width;    
    float virtual_width = 2*camera.screen_distance*tan(camera.fov/2 * PI/180);
    float virtual_height = aspect_ratio*virtual_width;
    float virtual_pixel_height = virtual_height/pixel_height;
    float virtual_pixel_width = virtual_width/pixel_width;

    Vec3 x_tmp = {1, 1, 0};
    Vec3 x_norm = grahamSchmidt(camera.direction, x_tmp, 1);
    Vec3 y_tmp = vecCross(camera.direction, x_norm);
    Vec3 y_norm = vecNormalize(y_tmp);

    Vec3 x_screen = vecScalarMult(x_norm, virtual_pixel_height);
    Vec3 y_screen = vecScalarMult(y_norm, virtual_pixel_width);
    Vec3 screen_center = vecScalarMult(camera.direction, camera.screen_distance);

    LightRay current_ray;
    current_ray.origin = camera.position;

    Vec3 x_scaled, y_scaled, tmp1, tmp2;
    LightRay reflected_ray;
    float hit_distance;
    
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
            current_ray.direction = vecNormalize(tmp2);
            
            current_ray.color = white;
            hit_distance = INFINITY;
            reflected_ray =  current_ray;

            int curr_obj_type; 
            // CHECK FOR COLLISIONS
            for (int i = 0; i < scene.obj_size; i++)
            {
                curr_obj_type = scene.obj_types[i];
                if (curr_obj_type == 0) // Sphere
                {
                    Sphere curr_obj = *((Sphere *)(((uint64_t* )scene.objects)[i])); 

                }
                else if (curr_obj_type == 1) // Plane
                {
                    Plane curr_obj = *((Plane *)(((uint64_t* )scene.objects)[i])); 
                }
                printf("%d\n", curr_obj_type);
            };

            

            *current_pixel_channel = reflected_ray.color.x; current_pixel_channel++;
            *current_pixel_channel = reflected_ray.color.y; current_pixel_channel++;
            *current_pixel_channel = reflected_ray.color.z; current_pixel_channel++;
        };
    };
};

/////////////////////////////////////////////////////////////////////////////
/* OUTPUT FUNCTIONS */
void array2ppm(unsigned char* image_arr, char filename[], int pixel_width, int pixel_height, int max_val)
{
    FILE *fp = fopen(filename, "wb");
    fprintf(fp, "P6\n%d %d\n%d\n", pixel_width, pixel_height, max_val);
    fwrite(image_arr, sizeof(unsigned char), pixel_width*pixel_height*3, fp);
    fclose(fp);
};


#endif