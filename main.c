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

typedef struct 
{
    VECTOR point;   
	SVECTOR normal; 
} Plane;

typedef struct
{
	VECTOR center;
	int radius;
	int16_t reflectivity;
} Sphere;

typedef struct 
{
    VECTOR position;
    SVECTOR color;
} Light;

//screenview variables
SVECTOR deltaU;
SVECTOR deltaV;
SVECTOR pix00; //upper left pixel, start point



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

inline SVECTOR PhongShading(Light* light, SVECTOR* dir, VECTOR* hitpoint, SVECTOR* hitnormal, SVECTOR* objcolor)
{
	//Vec3 ambient = vec_scale(object_color, 0.1);  // Componente ambientale

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
	NormFixed16(dir, &n);
	VecReflect(&lightdir, hitnormal, &reflectdir);
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

inline int RayTraceSphere(SVECTOR* orig, SVECTOR* dir, Sphere* sphere, int* t)
{
	VECTOR sphereorig32 = { sphere->center.vx, sphere->center.vy, sphere->center.vz };
	VECTOR orig32 = { orig->vx, orig->vy, orig->vz };
	VECTOR dir32 = { dir->vx, dir->vy, dir->vz };
	VECTOR oc;

	oc = sphere->center;

	//oc = VecInvertFixed32(&oc);

	int a = DotProductFixed32(&dir32, &dir32);
	int b = MulFixed32(DotProductFixed32(&dir32, &oc), -TWO);
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

inline SVECTOR TraceRayFixed(SVECTOR* orig, SVECTOR* dir, Plane* floor, Sphere* sphere, Light* light)
{
	SVECTOR color = { 0, 0, 0 };
	SVECTOR hitnormal;
	VECTOR hitpoint;
	//SVECTOR viewdir = VecInvertFixed16(&dir);// VecScaleFixed16(&dir, -ONE);

	int t;

	if (RayTraceSphere(orig, dir, sphere, &t))
	{
		SVECTOR objcolor = {ONE, ONE, 0};

		//trick 
		hitpoint.vx = MulFixed32(dir->vx, t) - sphere->center.vx;
		hitpoint.vy = MulFixed32(dir->vy, t) - sphere->center.vy;
		hitpoint.vz = MulFixed32(dir->vz, t) - sphere->center.vz;
		NormFixed32(&hitpoint, &hitnormal);

		color = PhongShading(light, dir, &hitpoint, &hitnormal, &objcolor);
	}
	else if (RayTracePlane(orig, dir, floor, &t))
	{
		// Colore del pavimento con Phong shading
		int16_t gray = FloatToFixed16(0.6f);
		//color = { gray, gray, gray };  // Grigio per il pavimento
		//color = { gray, gray, gray };  // Grigio per il pavimento
		//color = object_color;
		//color = phong_shading(hit_point, hit_normal, view_dir, light, object_color);
		//color = PhongShading(light, &hitpoint, &hitnormal, &color);
	}

	return color;
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
	VecSubFixed16(&focal, &viewportCompU, &viewportUL);
	VecSubFixed16(&viewportUL, &viewportCompV, &viewportUL);

	//pix start x0 y0
	SVECTOR pixCompUV;
	VecAddFixed16(&deltaU, &deltaV, &pixCompUV);
	VecScaleFixed16(&pixCompUV, HALF, &pixCompUV);
	VecAddFixed16(&viewportUL, &pixCompUV, &pix00);
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
	SVECTOR camera = {0, 0, 0};
	Sphere sphere = { {0, 0, -ONE}, HALF, HALF };
	Plane floor = {{0, -TEN, 0}, {0, -ONE, 0}};
	Light light = {{xlight, ylight, TWO}, {ONE, ONE, ONE}};

	SVECTOR posx = { 0, 0, 0 };
	SVECTOR posy = { 0, 0, 0 };
	SVECTOR dir;

	int x, y;

	for(y = 0; y < SCREEN_YRES; y++)
	{
		posx.vx = 0;
		posx.vy = 0;
		posx.vz = 0;

		for(x = 0; x < SCREEN_XRES; x++)
		{
			VecAddFixed16(&pix00, &posx, &dir);
			VecAddFixed16(&dir, &posy, &dir);

			SVECTOR color = TraceRayFixed(&camera, &dir, &floor, &sphere, &light);
			//SVECTOR color = raycolor(&normdir);

			//lowering precision to admit 0-255 range
			//from 9.7 to 16 int
			ptr->r = MulFixedVar(color.vx >> 5, MAX_GRAY, 7) >> 7;
			ptr->g = MulFixedVar(color.vy >> 5, MAX_GRAY, 7) >> 7;
			ptr->b = MulFixedVar(color.vz >> 5, MAX_GRAY, 7) >> 7;
			ptr++;

			VecAddFixed16(&posx, &deltaU, &posx);
		}

		VecAddFixed16(&posy, &deltaV, &posy);
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

	int xstart = FloatToFixed16(4);
	int ystart = FloatToFixed16(4);
	int xstep = FloatToFixed16(0.25f);
	int ystep = FloatToFixed16(0.25f);

	int x = xstart;
	int y = ystart;


	for (;;)
	{
		
		RayTrace(testBuffer, x, y);
		y -= ystep;

		if(y <= -FOUR)
			break;
	}

	y = ystart;
	x = xstart;

	for (;;)
	{
		
		RayTrace(testBuffer, x, y);
		x -= xstep;

		if(x <= -FOUR)
			break;
	}

	y = ystart;
	x = xstart;

	for (;;)
	{
		
		RayTrace(testBuffer, x, y);
		x -= xstep;
		y -= ystep;

		if(x <= -FOUR)
			break;
	}

	return 0;
}
