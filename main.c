/*
 * Fast Raytracing with Fixed Point Numbers 20.12 and 4.12
 * by SimoSbara 
 * insipired by https://raytracing.github.io/books/RayTracingInOneWeekend.html
 */

#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <psxgpu.h>
#include <psxetc.h>
#include <psxapi.h>
#include <psxgte.h>
#include <stdlib.h>
#include <psxcd.h>
#include <hwregs_c.h>
#include <inline_c.h>
#include "fixedmath.h"

#define SCREEN_XRES 320
#define SCREEN_YRES 240

typedef struct 
{
    uint8_t r, g, b;
} RGB;

uint8_t testBuffer[SCREEN_XRES * SCREEN_YRES * 3];

typedef struct _Plane 
{
    VECTOR point;   
	SVECTOR normal; 
} Plane;

typedef struct _Sphere
{
	VECTOR center;
	SVECTOR color;
	int radius;
	int16_t reflectivity;
} Sphere;

typedef struct _Light
{
    VECTOR position;
    SVECTOR color;
} Light;

typedef struct _Ray
{
	VECTOR origin;
	VECTOR direction;
} Ray;


//screenview variables
SVECTOR deltaU;
SVECTOR deltaV;
VECTOR pix00; //upper left pixel, start point
SVECTOR camera = {0, 0, 0};

//list of spheres
#define NUM_SPHERES	 3
Sphere spheres[NUM_SPHERES];
Sphere lightsphere  = { {0, 0, 0}, {ONE, ONE, ONE}, HALF, HALF };

//ambient light
SVECTOR ambient = {0, 0, 0};

// inline Vec3 phong_shading(Vec3 hit_point, Vec3 hit_normal, Vec3 view_dir, Light light, Vec3 object_color) 
// {

//     //Vec3 ambient = vec_scale(object_color, 0.1);  // Componente ambientale

//     // Componente diffusa
//     Vec3 light_dir = vec_normalize(vec_sub(light.position, hit_point));
//     float diff = fmax(vec_dot(hit_normal, light_dir), 0.0);
//     Vec3 diffuse = vec_scale(object_color, diff);
// 	/*
//     // Componente speculare
//     Vec3 reflect_dir = vec_reflect(vec_scale(light_dir, -1), hit_normal);
//     float spec = pow(fmax(vec_dot(view_dir, reflect_dir), 0.0), 16);
//     Vec3 specular = vec_scale(light.color, spec);

//     // Combinazione delle componenti
//     Vec3 color = vec_add(vec_add(ambient, diffuse), specular);
// 	*/
//     Vec3 color = diffuse;//vec_add(vec_add(ambient, diffuse), specular);
//     return color;
// }

inline SVECTOR PhongShading(Light* light, Ray* ray, VECTOR* hitpoint, SVECTOR* hitnormal, SVECTOR* objcolor)
{
	SVECTOR diffuse, specular, color;

    //diffuse
	SVECTOR lightdir;
	VECTOR sub;

	VecSubFixed32(&light->position, hitpoint, &sub);
	NormFixed32(&sub, &lightdir);
    
	int16_t diff = DotProductFixed16(&lightdir, hitnormal);

	if(diff < 0)
		diff = 0;

    VecScaleFixed16(objcolor, diff, &diffuse);

    //specular
	SVECTOR n, reflectdir;
	NormFixed32(&ray->direction, &n);
	VecReflect16(&lightdir, hitnormal, &reflectdir);
	NormFixed16(&reflectdir, &reflectdir);

	int dotspec = DotProductFixed16(&n, &reflectdir);
	int spec = 0;

	if(dotspec < 0)
		dotspec = 0;
	else if(dotspec > 0)
		spec = pow(dotspec, 16);

	VecScaleFixed16(&light->color, spec, &specular);

	//combining diffuse and specular
	VecAddFixed16(&diffuse, &specular, &color);
	//VecAddFixed16(&color, &ambient, &color);

	color.vx = min(color.vx, ONE);
	color.vy = min(color.vy, ONE);
	color.vz = min(color.vz, ONE);

	return color;
}

inline int RayTracePlane(SVECTOR* orig, SVECTOR* dir, Plane* plane, int* t)
{
	int eps = 1; // 2^-11

	int denom = DotProductFixed16(&plane->normal, dir);

	if (denom > eps)
	{
		SVECTOR p0l0s;
		VECTOR or, p0l0;
		or.vx = orig->vx;
		or.vy = orig->vy;
		or.vz = orig->vz;

		VecSubFixed32(&plane->point, &or, &p0l0);
		
		p0l0s.vx = p0l0.vx;
		p0l0s.vy = p0l0.vy;
		p0l0s.vz = p0l0.vz;

		//SVECTOR p0l0 = VecSubFixed16(&plane.point, &orig);
		*t = DivFixed(DotProductFixed16(&p0l0s, &plane->normal), denom);

		// if (t >= 0)
		// {
		// 	//*hit_point = vec_add(orig, vec_scale(dir, *t));
		// 	//*hit_normal = plane.normal;
		// 	return 1;
		// }

		return 1;
	}
	return 0;
}

inline int RayTraceSphere(Ray* ray, Sphere* sphere, int* t)
{
	VECTOR oc;

	VecSubFixed32(&sphere->center, &ray->origin, &oc);

	int a = DotProductFixed32(&ray->direction, &ray->direction);
	int b = MulFixed32(DotProductFixed32(&ray->direction, &oc), -TWO);
	int c = DotProductFixed32(&oc, &oc) - MulFixed(sphere->radius, sphere->radius);

	int d1 = MulFixed32(b, b);
	int d2 = MulFixed32(a, c);
	int d3 = MulFixed32(-FOUR, d2);

	int delta = d1 + d3;

	if (delta == 0)
	{
		*t = DivFixed32(-b, MulFixed32(TWO, a));
		return 1;
	}
	else if (delta > 0)
	{
		int rad = SquareRoot12(delta);
		int num = -b - rad;
		*t = DivFixed32(num, MulFixed32(TWO, a));

		return 1;
	}
	
	return 0;
}

#define MAX_DEPTH	1

int TraceRayFixed(Ray* ray, Plane* floor, Light* light, SVECTOR* color, int depth)
{
	SVECTOR hitnormal;
	VECTOR hitpoint;
	int t;

	// if (RayTraceSphere(orig, dir, &lightsphere, &t))
	// 	return lightsphere.color;

	for(int i = 0; i < NUM_SPHERES; i++)
	{
		Sphere* s = &spheres[i];

		if (RayTraceSphere(ray, s, &t))
		{
			//trick 
			VecScaleFixed32(&ray->direction, t, &hitpoint);
			VecSubFixed32(&hitpoint, &s->center, &hitpoint);
			NormFixed32(&hitpoint, &hitnormal);

			if(depth < MAX_DEPTH)
			{
				Ray reflected;
				reflected.origin = hitpoint;
				VecReflect32(&ray->direction, &hitnormal, &reflected.direction);

				if(TraceRayFixed(&reflected, floor, light, color, depth + 1))
				{
					//VecScaleFixed16(&color, s->reflectivity, &color);
					//SVECTOR fcolor = {ONE, ONE, 0};

					//color->vx = ONE;
					//color->vy = ONE;
					//color->vz = 0;

					return 1;
				}
			}
			else
				*color = s->color;
			
			*color = PhongShading(light, ray, &hitpoint, &hitnormal, color);

			return 1;//PhongShading(light, ray, &hitpoint, &hitnormal, &color);
		}
		// else if (RayTracePlane(orig, dir, floor, &t))
		// {
		// 	// Colore del pavimento con Phong shading
		// 	int16_t gray = FloatToFixed16(0.6f);
		// 	//color = { gray, gray, gray };  // Grigio per il pavimento
		// 	//color = { gray, gray, gray };  // Grigio per il pavimento
		// 	//color = object_color;
		// 	//color = phong_shading(hit_point, hit_normal, view_dir, light, object_color);
		// 	//color = PhongShading(light, &hitpoint, &hitnormal, &color);
		// }
	}

	*color = ambient;

	return 0;
}

SVECTOR raycolor(SVECTOR* dir)
{
	//vec3 unit_direction = unit_vector(r.direction());

	SVECTOR color;

	//SVECTOR normdir = NormFixed16(dir->vx, dir->vy, dir->vz);

	int a = MulFixed(dir->vy + ONE, HALF);
	int a1 = MulFixed(a, HALF);
	int a2 = MulFixed(a, FloatToFixed16(0.7f));
	int a3 = MulFixed(a, ONE);

	color.vx = ONE - a + a1;
	color.vy = ONE - a + a2;
	color.vz = ONE - a + a3;


	// float ny = FixedToFloat(dir->vy);
	// float a = 0.5f * (ny + 1.0f);

	// color.vx = FloatToFixed16(1.0f - a + a * 0.5f);
	// color.vy = FloatToFixed16(1.0f - a + a * 0.7f);
	// color.vz = FloatToFixed16(1.0f - a + a * 1.0f);


	return color;
}


void PrepareCamera(int w, int h)
{
	float ar = (float)w / h;

	float focalCam = 1.0f;//1.0;
	float vh = 2.0f;
	float vw = vh * ar;

	SVECTOR viewportU = { FloatToFixed16(vw), 0, 0 };
	SVECTOR viewportV = { 0, FloatToFixed16(-vh), 0 };

	int16_t invW = FloatToFixed16(1.0f / w);
	int16_t invH = FloatToFixed16(1.0f / h);

	//global
	VecScaleFixed16(&viewportU, invW, &deltaU);
	VecScaleFixed16(&viewportV, invH, &deltaV);

	//upper left
	SVECTOR viewportUL, viewportCompU, viewportCompV;
	SVECTOR focal = { 0, 0, -FloatToFixed16(focalCam)};

	VecScaleFixed16(&viewportU, HALF, &viewportCompU);
	VecScaleFixed16(&viewportV, HALF, &viewportCompV);
	VecSubFixed16(&camera, &focal, &viewportUL);
	VecSubFixed16(&focal, &viewportCompU, &viewportUL);
	VecSubFixed16(&viewportUL, &viewportCompV, &viewportUL);

	//pix start x0 y0
	SVECTOR pixCompUV, pix00s;
	VecAddFixed16(&deltaU, &deltaV, &pixCompUV);
	VecScaleFixed16(&pixCompUV, HALF, &pixCompUV);
	VecAddFixed16(&viewportUL, &pixCompUV, &pix00s);

	Vec16ToVec32(&pix00s, &pix00);
}

void RayTrace(void* buffer, int xlight, int ylight)
{
	RGB* ptr = buffer;

	RECT rect;
	rect.x = 0;
	rect.y = 0;
	rect.w = (SCREEN_XRES / 2) * 3;
	rect.h = SCREEN_YRES;

	int16_t zsphere = FloatToFixed16(-3 - rand() % 3);

	//int16_t zsphere = FloatToFixed16(rand() % 3 - 7);
	Plane floor = {{0, -TEN, 0}, {0, -ONE, 0}};
	Light light = {{xlight, ylight, ONE}, {ONE, ONE, ONE}};

	VECTOR posx = { 0, 0, 0 };
	VECTOR posy = { 0, 0, 0 };
	SVECTOR color;
	Ray ray;

	int x, y;

	lightsphere.center = light.position;
	lightsphere.color = light.color;

	Vec16ToVec32(&camera, &ray.origin);

	for(y = 0; y < SCREEN_YRES; y++)
	{
		posx.vx = 0;
		posx.vy = 0;
		posx.vz = 0;

		for(x = 0; x < SCREEN_XRES; x++)
		{
			VecAddFixed32(&pix00, &posx, &ray.direction);
			VecAddFixed32(&ray.direction, &posy, &ray.direction);

			TraceRayFixed(&ray, &floor, &light, &color, 1);

			//lowering precision to admit 0-255 range
			//from 9.7 to 16 int
			ptr->r = MulFixedVar(color.vx >> 5, MAX_GRAY, 7) >> 7;
			ptr->g = MulFixedVar(color.vy >> 5, MAX_GRAY, 7) >> 7;
			ptr->b = MulFixedVar(color.vz >> 5, MAX_GRAY, 7) >> 7;
			ptr++;

			posx.vx += deltaU.vx;
			posx.vy += deltaU.vy;
			posx.vz += deltaU.vz;
		}

		posy.vx += deltaV.vx;
		posy.vy += deltaV.vy;
		posy.vz += deltaV.vz;
	}
	
	LoadImage(&rect, buffer);

	DrawSync(0);
	VSync(0);
}

int main(int argc, const char **argv) 
{
	// Initialize the GPU and load the default font texture provided by
	// PSn00bSDK at (960, 0) in VRAM.
	ResetGraph(0);

	DISPENV		disp;

	SetDefDispEnv(&disp, 0, 0, SCREEN_XRES, SCREEN_YRES);
	disp.isrgb24 = 1;
	disp.isinter = 1;

	//setting display
	PutDispEnv(&disp);
	SetDispMask(1);

	//inizializzo coprocessore GEOMETRY TRANSFORMATION ENGINE
	InitGeom();
	//gte_SetGeomOffset( CENTERX, CENTERY );
	//gte_SetGeomScreen( CENTERX );

	// Set up controller polling.
	//uint8_t pad_buff[2][34];
	//InitPAD(pad_buff[0], 34, pad_buff[1], 34);
	//StartPAD();
	//ChangeClearPAD(0);

	PrepareCamera(SCREEN_XRES, SCREEN_YRES);

	Sphere s1 = { {-HALF, 0, -TWO}, {0, ONE, 0}, HALF, HALF };
	Sphere s2 = { {HALF, 0, -TWO}, {ONE, 0, 0}, HALF, HALF };
	Sphere s3 = { {0, ONE, -TWO}, {0, 0, ONE}, HALF, HALF };
	spheres[0] = s1;
	spheres[1] = s2;
	spheres[2] = s3;

	lightsphere.radius = FixedToFloat(0.1f);

	while(true)
	{
		int xstart = FloatToFixed16(4);
		int ystart = FloatToFixed16(4);
		int xstep = FloatToFixed16(0.25f);
		int ystep = FloatToFixed16(0.25f);

		int x = xstart;
		int y = ystart;

		for (;;)
		{
			RayTrace(testBuffer, x, y);
			x -= xstep;
			y -= ystep;

			if(x <= -FOUR)
				break;
		}
	}

	return 0;
}
