#ifndef linalg
#define linalg
#define ABS(x) ((x) > 0 ? (x) : -(x))

#include <math.h>
#include <assert.h>

/////////////////////////////////////////////////////////////////////////////
/* BASIC ELEMENTS */

typedef struct 
{
    float x;
    float y;
    float z; 
} Vec3;


typedef struct
{
    Vec3 row1;
    Vec3 row2;
    Vec3 row3;
} Matrix3;
/////////////////////////////////////////////////////////////////////////////
/* VECTOR OPERATIONS */

int vecEqual(Vec3 vec1, Vec3 vec2)
{
    return ((ABS(vec1.x - vec2.x)<1e-6) & (ABS(vec1.y - vec2.y)<1e-6) & (ABS(vec1.z - vec2.z)<1e-6));
}


float vecDot(Vec3 vec1, Vec3 vec2)
{
    return (vec1.x * vec2.x) + (vec1.y * vec2.y) + (vec1.z * vec2.z);
};


float vecMagnitude(Vec3 vec)
{
    return sqrt(vecDot(vec, vec));
};


Vec3 vecScalarMult(Vec3 vec, float scalar)
{
    Vec3 result; 
    result.x = vec.x * scalar;
    result.y = vec.y * scalar;
    result.z = vec.z * scalar;
    return result;
}


Vec3 vecNormalize(Vec3 vec)
{
    float mag = vecMagnitude(vec);
    Vec3 norm_vec = vecScalarMult(vec, 1/mag);
    return norm_vec;
};


Vec3 vecAdd(Vec3 vec1, Vec3 vec2)
{
    Vec3 vec3;
    vec3.x = vec1.x + vec2.x;
    vec3.y = vec1.y + vec2.y;
    vec3.z = vec1.z + vec2.z;
    return vec3;
}


Vec3 vecCross(Vec3 vec1, Vec3 vec2)
{
    Vec3 vec3;
    vec3.x = vec1.y * vec2.z - vec2.y * vec1.z;
    vec3.y = vec1.z * vec2.x - vec2.z * vec1.x;
    vec3.z = vec1.x * vec2.y - vec2.x * vec1.y;
    return vec3;
}


Vec3 vec3ComponentMult(Vec3 vec1, Vec3 vec2)
{
    Vec3 vec3 = {vec1.x*vec2.x, vec1.y*vec2.y, vec1.z*vec2.z};
    return vec3;
}


float vec3Max(Vec3 vec)
{
    if (vec.x > vec.y)
    {
        if (vec.x > vec.z) return vec.x;
        return vec.z;
    } 
    if (vec.y > vec.x)
    {    
        if (vec.y > vec.z) return vec.y;
        return vec.z;
    } 
    if (vec.x > vec.z) return vec.x;
    return vec.z;
}

Vec3 vec3Clamp(Vec3 vec, float min, float max)
{
    vec.x = (vec.x < min) ? min : (vec.x > max) ? max : vec.x;
    vec.y = (vec.y < min) ? min : (vec.y > max) ? max : vec.y;
    vec.z = (vec.z < min) ? min : (vec.z > max) ? max : vec.z;
    return vec;
}
/////////////////////////////////////////////////////////////////////////////
/* MATRIX OPERATIONS */

float mat3Determinant(Matrix3* A)
{
    return A->row1.x*(A->row2.y*A->row3.z-A->row2.z*A->row3.y) - A->row1.y*(A->row2.x+A->row3.z - A->row2.z*A->row3.x) + A->row1.z*(A->row2.x*A->row3.y - A->row3.x*A->row2.y);
};


Matrix3 mat3ScalarMult(Matrix3 mat, float a)
{
    mat.row1 = vecScalarMult(mat.row1, a);
    mat.row2 = vecScalarMult(mat.row2, a);
    mat.row3 = vecScalarMult(mat.row3, a);
    return mat;
};

Matrix3 mat3Inverse(Matrix3* A)
{
    float C11 = A->row2.y*A->row3.z - A->row2.z*A->row3.y;
    float C12 = A->row2.x*A->row3.z - A->row2.z*A->row3.x;
    float C13 = A->row2.x*A->row3.y - A->row2.y*A->row3.x;
    float C21 = A->row1.y*A->row3.z - A->row1.z*A->row3.y;
    float C22 = A->row1.x*A->row3.z - A->row1.z*A->row3.x;
    float C23 = A->row1.x*A->row3.y - A->row1.y*A->row3.x;
    float C31 = A->row1.y*A->row2.z - A->row1.z*A->row3.x;
    float C32 = A->row1.x*A->row2.z - A->row1.z*A->row2.x;
    float C33 = A->row1.x*A->row2.y - A->row1.y*A->row2.x;
    float det = A->row1.x * C11 + A->row1.y * C12 + A->row1.z * C13;
    assert(abs(det) > 1e-5);
    Matrix3 CofMat = {.row1 = {C11, C12, C13}, .row2 = {C21, C22, C23}, .row3 = {C31, C32, C33}};
    Matrix3 invA;
    return invA;
};

/////////////////////////////////////////////////////////////////////////////
/* OTHER FUNCTIONS */

Vec3 grahamSchmidt(Vec3 norm_vec, Vec3 unnormed_vec, int norm_res)
{
    Vec3 projection = vecScalarMult(norm_vec, vecDot(norm_vec, unnormed_vec));
    Vec3 neg_projection = vecScalarMult(projection, -1.0F);
    Vec3 perp_vec = vecAdd(unnormed_vec, neg_projection);
    if (!norm_res) return perp_vec;
    return vecNormalize(perp_vec);   
}

#endif