#include <stdint.h>
#include <psxgte.h>

//9.7 fixed point
const int16_t MAX_GRAY = 255 << 7; 

//4.12 fixed point	
const int16_t HALF = 1 << 11;
const int16_t TWO = 2 << 12;
const int16_t THREE = 3 << 12;
const int16_t FOUR = 4 << 12;
const int16_t FIVE = 5 << 12;
const int16_t TEN = 10 << 12;

//copy
inline void Vec32ToVec16(VECTOR* v1, SVECTOR* v2)
{
    v2->vx = v1->vx;
    v2->vy = v1->vy;
    v2->vz = v1->vz;
}

//copy
inline void Vec16ToVec32(SVECTOR* v1, VECTOR* v2)
{
    v2->vx = v1->vx;
    v2->vy = v1->vy;
    v2->vz = v1->vz;
}

//fixed n.12 to float 
inline float FixedToFloat(int input)
{
	return ((float)input / (float)(1 << 12));
}

//da 4.12 a 20.12 fixed
inline int MulFixed(int16_t a, int16_t b)
{
	return (int)(((int)(a) * (int)(b)) >> 12);
}

//20.12 fixed
inline int MulFixed32(int a, int b)
{
	return (int)(((int64_t)(a) * (int64_t)(b)) >> 12);
}


//da 4.12 a 32-dp.dp fixed
inline int MulFixedVar(int16_t a, int16_t b, int dp)
{
	return (int)(((int)(a) * (int)(b)) >> dp);
}

//da 4.12 a 20.12 fixed
inline int DivFixed(int16_t a, int16_t b)
{
	return ((int)(a) << 12) / (int)(b);
}

//da 4.12 a 20.12 fixed
inline int16_t DivFixed32(int a, int b)
{
	return ((int16_t)(a) << 12) / (int16_t)(b);
}


//20.12 fixed
inline int FloatToFixed32(float num)
{
	return num * (float)(1 << 12) + (num >= 0 ? 0.5 : - 0.5);
}

//4.12 fixed
inline int16_t FloatToFixed16(float num)
{
	return num * (float)(1 << 12) + (num >= 0 ? 0.5 : - 0.5);
}

inline int16_t Complement2(int16_t a)
{
	int16_t b = ~a;
	b++;

	return b;
}

inline void VecInvertFixed16(SVECTOR* v, SVECTOR* r)
{
	r->vx = Complement2(v->vx);
	r->vy = Complement2(v->vy);
	r->vz = Complement2(v->vz);
}

inline void VecInvertFixed32(VECTOR* v, VECTOR* r)
{
	r->vx = Complement2(v->vx);
	r->vy = Complement2(v->vy);
	r->vz = Complement2(v->vz);
}

inline void VecScaleFixed16(SVECTOR* v, int16_t scale, SVECTOR* r)
{
	r->vx = MulFixed(v->vx, scale);
	r->vy = MulFixed(v->vy, scale);
	r->vz = MulFixed(v->vz, scale);
}

inline void VecScaleFixed32(VECTOR* v, int scale, VECTOR* r)
{
	r->vx = MulFixed32(v->vx, scale);
	r->vy = MulFixed32(v->vy, scale);
	r->vz = MulFixed32(v->vz, scale);
}

inline void VecAddFixed16(SVECTOR* v1, SVECTOR* v2, SVECTOR* r)
{
	r->vx = v1->vx + v2->vx;
	r->vy = v1->vy + v2->vy;
	r->vz = v1->vz + v2->vz;
}

inline void VecAddFixed32(VECTOR* v1, VECTOR* v2, VECTOR* r)
{
	r->vx = v1->vx + v2->vx;
	r->vy = v1->vy + v2->vy;
	r->vz = v1->vz + v2->vz;
}

inline void VecSubFixed16(SVECTOR* v1, SVECTOR* v2, SVECTOR* r)
{
	r->vx = v1->vx - v2->vx;
	r->vy = v1->vy - v2->vy;
	r->vz = v1->vz - v2->vz;
}

inline void VecSubFixed32(VECTOR* v1, VECTOR* v2, VECTOR* r)
{
	r->vx = v1->vx - v2->vx;
	r->vy = v1->vy - v2->vy;
	r->vz = v1->vz - v2->vz;
}

inline int DotProductFixed16(SVECTOR* v1, SVECTOR* v2)
{
	return MulFixed(v1->vx, v2->vx) + MulFixed(v1->vy, v2->vy) + MulFixed(v1->vz, v2->vz);
}

inline int DotProductFixed32(VECTOR* v1, VECTOR* v2)
{
	return MulFixed32(v1->vx, v2->vx) + MulFixed32(v1->vy, v2->vy) + MulFixed32(v1->vz, v2->vz);
}

inline void NormFixed16(SVECTOR* v, SVECTOR* n)
{
	int16_t den = SquareRoot12(DotProductFixed16(v, v));
	
	if(den == 0)
	{
		n->vx = 0;
		n->vy = 0;
		n->vz = 0;
	}
	else
	{
		n->vx = DivFixed(v->vx, den);
		n->vy = DivFixed(v->vy, den);
		n->vz = DivFixed(v->vz, den);
	}
}

inline void NormFixed32(VECTOR* v, SVECTOR* n)
{
	int den = SquareRoot12(DotProductFixed32(v, v));

	if(den == 0)
	{
		n->vx = 0;
		n->vy = 0;
		n->vz = 0;
	}
	else
	{
		n->vx = DivFixed(v->vx, den);
		n->vy = DivFixed(v->vy, den);
		n->vz = DivFixed(v->vz, den);
	}
}

//a ray reflects from the normal of the surface
inline void VecReflect16(SVECTOR* v, SVECTOR* n, SVECTOR* r) 
{
    SVECTOR nscaled;
	int scaling = MulFixed(TWO, DotProductFixed16(v, n));
	VecScaleFixed16(n, scaling, &nscaled);
    VecSubFixed16(v, &nscaled, r);
}

inline void VecReflect32(VECTOR* v, SVECTOR* n, VECTOR* r) 
{
    VECTOR nscaled;
    VECTOR n32;

    Vec16ToVec32(n, &n32);

	int scaling = MulFixed(TWO, DotProductFixed32(v, &n32));
	VecScaleFixed32(&n32, scaling, &nscaled);
    VecSubFixed32(v, &nscaled, r);
}



inline int max(int a, int b)
{ 
    return (a > b) ? a : b;
}

inline int min(int a, int b)
{
    return (a < b) ? a : b;
}

inline int pow(int a, int b)
{
	int i;
	int c = a;
	
	for(i = 0; i < b; i++)
	{
		c = MulFixed32(c, a);
	}

	return c;
}