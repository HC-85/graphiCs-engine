#ifndef graphengine
#define graphengine
#define PI 3.1415926535
#define MAX_QUEUE_SIZE 1000
#define MAX_TREE_SIZE 10000
#define RED_WAVELENGTH 0.685
#define BLUE_WAVELENGTH 0.4725
#define GREEN_WAVELENGTH 0.5145
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
    Ray ray;
    float wavelength; // micrometers
} frLightRay;

typedef struct
{
    float ambient_ref;
    float diffuse_ref;
    float specular_ref;
    float shininess; 
    float reflectiveness;
    Vec3 color;
} phMaterial;

typedef struct{
    float B1;
    float B2;
    float B3;
    float C1;
    float C2;
    float C3;
} SellmeierCoeffs;

typedef struct
{
    float ambient_ref;
    float diffuse_ref;
    float specular_ref;
    float shininess; 
    float reflectiveness;
    Vec3 color;
    SellmeierCoeffs sellmeier_coeffs;
} frMaterial;

typedef struct
{
    void* objects;
    void* lighting;
    int* obj_types;
    int* light_types;
    int obj_size;
    int light_size; 
} Scene;

typedef struct _RayTreeNode
{
    struct _RayTreeNode* parent;
    Vec3 reflected_shoot;
    Vec3 transmitted_shoot;
    Vec3 position;
    Vec3 color;
    Vec3 normal;
    int depth;
    int scene_obj_ind;
    float refraction_index;
} RayTreeNode;

typedef struct
{
    RayTreeNode* buds[MAX_QUEUE_SIZE];
    int front;
    int back;
} RayQueue;

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
    phLightRay (*tracing_function)(phProtoSphere, phMaterial, phLightRay, Scene, int);
} phSphere;

typedef struct
{
    phProtoSphere proto;
    frMaterial material;
    float (*hit_function)(phProtoSphere, Ray);
    phLightRay (*tracing_function)(phProtoSphere, phMaterial, phLightRay, Scene, int);
} frSphere;

typedef struct
{
    phProtoPlane proto;
    phMaterial material;
    float (*hit_function)(phProtoPlane, phLightRay);
    phLightRay (*tracing_function)(phProtoPlane, phMaterial, phLightRay, Scene, int);
} phPlane;

typedef struct
{
    phProtoPlane proto;
    frMaterial material;
    float (*hit_function)(phProtoPlane, Ray);
    phLightRay (*tracing_function)(phProtoPlane, phMaterial, phLightRay, Scene, int);
} frPlane;

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
/* QUEUE FUNCTIONS */
int joinQueue(RayQueue* queue, RayTreeNode* new_bud)
{
    if (((queue->back)-(queue->front))%MAX_QUEUE_SIZE == 1) return -1; // QUEUE FULL
    queue->buds[queue->back] = new_bud;
    if (queue->back == (MAX_QUEUE_SIZE - 1)) queue->back = 0; //ROLLBACK
    else queue->back ++;
    return 0;
}

RayTreeNode* leaveQueue(RayQueue* queue){
    if (queue->front == queue->back) return -1; // QUEUE EMPTY
    RayTreeNode* front_ray = queue->buds[queue->front];
    if (queue->front == (MAX_QUEUE_SIZE - 1)) queue->front = 0; //ROLLBACK
    else queue->front ++;
    return front_ray;
}

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


Vec3 phongIllumination2(Vec3 ambient_color, phMaterial material, Scene scene, Vec3 intersection, Vec3 surf2cam_direction, Vec3 surf_normal, Vec3 reflected_direction, int reflection_depth)
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


phLightRay phongSphereTracing2(phProtoSphere sphere, phMaterial material, phLightRay lightray, Scene scene, int current_tracing_depth){
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
    Vec3 surf2cam_direction = vecScalarMult(vecNormalize(cam2surf), -1);

    Vec3 illum = phongIllumination2((Vec3) {255,255,255}, material, scene, intersection, surf2cam_direction, surf_normal, reflected_direction, current_tracing_depth);
    Vec3 newcolor = vec3Clamp(vecAdd(lightray.color, vecScalarMult(illum, pow(current_tracing_depth, -1/material.reflectiveness))), 0, 255);
    phLightRay reflected_lightray = {.ray = {.origin = intersection, .direction = reflected_direction}, .color = newcolor};
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


phLightRay phongFloorTracing2(phProtoPlane floorplane, phMaterial material, phLightRay lightray, Scene scene, int current_tracing_depth)
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
    Vec3 illum = phongIllumination2(white, material, scene, intersection, vecNormalize(cam2surf), floorplane.direction, reflected_direction, current_tracing_depth);
    Vec3 newcolor = vec3Clamp(vecAdd(lightray.color, vecScalarMult(illum, pow(current_tracing_depth, -1/material.reflectiveness))), 0, 255);

    phLightRay reflected_lightray = {.ray = {.origin = intersection, .direction = reflected_direction}, .color = newcolor};
    return reflected_lightray;
};

/////////////////////////////////////////////////////////////////////////////
/* META FUNCTIONS */

float findIntersect(Ray ray, Scene scene, int* obj_ind){
    float closest_dist = INFINITY;
    float intersect_dist;
    int closest_ind;
    for (int i = 0; i < scene.obj_size; i++)
    {
        int curr_obj_type = scene.obj_types[i];
        if (curr_obj_type == 0) // Sphere
        {
            frSphere curr_obj = *((frSphere*)(((uint64_t*)scene.objects)[i]));
            intersect_dist = curr_obj.hit_function(curr_obj.proto, ray);
        }
        else if (curr_obj_type == 1) // Plane
        {
            frPlane curr_obj = *((frPlane*)(((uint64_t*)scene.objects)[i])); 
            intersect_dist = curr_obj.hit_function(curr_obj.proto, ray);
        }

        if (intersect_dist<closest_dist) // SAVE CLOSEST INTERSECTION
        {
            closest_dist = intersect_dist;
            closest_ind = i;
        }
    };
    *obj_ind = closest_ind;
    return intersect_dist;
};


Vec3 findNormal(RayTreeNode* previous_node, RayTreeNode* current_node, Scene* scene){
    int obj_type = scene->obj_types[current_node->scene_obj_ind];
    Vec3 surf_normal;
    if (obj_type == 0)
    {
        frSphere sphere = *((frSphere*)(((uint64_t*)scene->objects)[current_node->scene_obj_ind]));
        surf_normal = vecNormalize(vecAdd(current_node->position, vecScalarMult(sphere.proto.center, -1)));
    }
    else if (obj_type == 1)
    {
        frPlane plane = *((frPlane*)(((uint64_t*)scene->objects)[current_node->scene_obj_ind]));
        surf_normal = plane.proto.direction;
    }
    return surf_normal;
};


Vec3 findReflection(RayTreeNode* previous_node, RayTreeNode* current_node){
    Vec3 incident_ray = vecNormalize(vecAdd(current_node->position, vecScalarMult(previous_node->position, -1)));
    Vec3 flip_vector = vecScalarMult(current_node->normal, -2*vecDot(incident_ray, current_node->normal));
    Vec3 surf_reflection = vecAdd(incident_ray, flip_vector);
    surf_reflection = vecNormalize(surf_reflection);
    return surf_reflection;
};


float sellmeierDispersion(float wavelength, SellmeierCoeffs sellmeier_coeffs){
    // For glasses
    // Wavelength in micrometers
    float wl2 = wavelength*wavelength;
    float n2 = 1 + (sellmeier_coeffs.B1*wl2)/(wl2-sellmeier_coeffs.C1) + (sellmeier_coeffs.B2*wl2)/(wl2-sellmeier_coeffs.C2) + (sellmeier_coeffs.B3*wl2)/(wl2-sellmeier_coeffs.C3);
    return sqrt(n2);
};


Vec3 findTransmission(RayTreeNode* previous_node, RayTreeNode* current_node, Scene* scene, float wavelength){
    int inc_ref_ind = previous_node->refraction_index;
    
    int trans_obj_type = scene->obj_types[current_node->scene_obj_ind];
    frMaterial material;
    if (trans_obj_type == 0)
    {
        frSphere sphere = *((frSphere*)(((uint64_t*)scene->objects)[current_node->scene_obj_ind]));
        material = sphere.material;
    }
    else if (trans_obj_type == 1)
    {
        frPlane plane = *((frPlane*)(((uint64_t*)scene->objects)[current_node->scene_obj_ind]));
        material = plane.material;
    }

    float trans_ref_ind = sellmeierDispersion(wavelength, material.sellmeier_coeffs);

    Vec3 incident = vecNormalize(vecAdd(current_node->position, vecScalarMult(previous_node->position,-1)));
    float y_inc = vecDot(incident, current_node->normal);
    Vec3 perp2normal = vecAdd(incident, vecScalarMult(current_node->normal, y_inc));
    float x_inc = vecMagnitude(perp2normal);
    perp2normal = vecNormalize(perp2normal);
    float x_trans = x_inc*inc_ref_ind/trans_ref_ind;
    float y_trans = sqrt(1 - x_trans*x_trans);
    Vec3 transmitted = vecAdd(vecScalarMult(perp2normal, x_trans), vecScalarMult(current_node->normal, -y_trans));
    assert((1 - ABS(vecMagnitude(transmitted))) < 1e-3);
    return transmitted;
}


int fresnelForwardTracing(frLightRay rootray, Scene scene){
    // BEWARE OF COPYING, MAY NEED TO CHANGE FUNCTIONS TO PASS BY REFERENCE
    // create tree
    RayTreeNode* raytree = (RayTreeNode *) malloc(MAX_TREE_SIZE * sizeof(RayTreeNode));
    RayTreeNode* tree_ptr = raytree;

    // create rootnode
    RayTreeNode rootnode = {
        .parent = NULL, 
        .position = rootray.ray.origin,
        .depth = 0,
        .scene_obj_ind = NULL,
        .refraction_index = 1};
    
    *tree_ptr = rootnode; tree_ptr++;
    // find closest intersection
    int closest_obj_ind;
    float intersect_dist = findIntersect(rootray.ray, scene, &closest_obj_ind);
    if (intersect_dist > 1e8) 
    {
        free(raytree);
        return 1;
    }
    // create first child
    Vec3 intersection = vecAdd(rootnode.position, vecScalarMult(rootray.ray.direction, -intersect_dist));
    RayTreeNode trunk = {
        .parent = &rootnode,
        .position = intersection,
        .scene_obj_ind = closest_obj_ind,
        .depth = rootnode.depth + 1
    };
    *tree_ptr = trunk; tree_ptr++;
    // find normal
    trunk.normal = findNormal(&rootnode, &trunk, &scene);
    // initialize queue
    RayQueue growthqueue = {.back = 0, .front = 0};
    // add trunk to queue
    if (joinQueue(&growthqueue, &trunk)<0) printf("FULL QUEUE");
    // get reflection sprout direction
    trunk.reflected_shoot = findReflection(&rootnode, &trunk);
    // get transmission sprout direction
    trunk.transmitted_shoot = findTransmission(&rootnode, &trunk, &scene, RED_WAVELENGTH);
    // check queue for bud-growth
    RayTreeNode* current_bud = leaveQueue(&growthqueue);
    // shoot reflected current bud
    assert((1-ABS(vecMagnitude(current_bud->reflected_shoot)))<1e-3);
    Ray current_ray = {.origin = current_bud->position, .direction = current_bud->reflected_shoot};
    intersect_dist = findIntersect(current_ray, scene, &closest_obj_ind);
    intersection = vecAdd(rootnode.position, vecScalarMult(rootray.ray.direction, -intersect_dist));
    printf("x: %f, y: %f, z: %f\n", intersection.x, intersection.y, intersection.z);
    // shoot transmitted current bud
    assert((1-ABS(vecMagnitude(current_bud->transmitted_shoot)))<1e-3);
    current_ray.origin = current_bud->position;
    current_ray.direction = current_bud->transmitted_shoot;
    intersect_dist = findIntersect(current_ray, scene, &closest_obj_ind);
    intersection = vecAdd(rootnode.position, vecScalarMult(rootray.ray.direction, -intersect_dist));
    printf("x: %f, y: %f, z: %f\n", intersection.x, intersection.y, intersection.z);
    free(raytree);
    return 42;
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
/* TRACING MANAGERS */
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


void fresnelTracing(){
    // Forward Tracing
    // Backward Coloring
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