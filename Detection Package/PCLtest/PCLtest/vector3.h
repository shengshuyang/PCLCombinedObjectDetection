#include <Windows.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>
#include <vector>
#include "picLibrary\pic.h"

class vector3
{
public:
	GLfloat x;
	GLfloat y;
	GLfloat z;
	vector3(){};
	vector3(GLfloat ix, GLfloat iy, GLfloat iz){ x = ix; y = iy; z = iz;};

	vector3*  cross(vector3* v2);
	vector3*  add  (vector3* v2);
	vector3*  sub  (vector3* v2);
	void operator=(vector3* i){ x = i->x;y = i->y;z = i->z;};
	void normalize();
};

vector3* vector3::cross(vector3* v2)
{
	// i   j   k  
	// 1x  1y  1z
	// 2x  2y  2z
	vector3* output = new vector3();
	output->x = y * v2->z - v2->y * z;
	output->y =-x * v2->z + v2->x * z;
	output->z = x * v2->y - v2->x * y;
	return output;
}

	vector3* vector3::add(vector3* v2)
	{
		vector3* output = new vector3();
		output->x = x + v2->x;
		output->y = y + v2->y;
		output->z = z + v2->z;
		return output;
	}

	vector3* vector3::sub(vector3* v2)
	{
		vector3* output = new vector3();
		output->x = x - v2->x;
		output->y = y - v2->y;
		output->z = z - v2->z;
		return output;
	}

	void vector3::normalize()
	{
		double r = sqrt(x*x + y*y + z*z);
		if(r != 0)
		{
			x/=r;
			y/=r;
			z/=r;
		}
	}