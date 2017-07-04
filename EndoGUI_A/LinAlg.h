#pragma once
#ifndef LINALG_H
#define LINALG_H

#include <math.h>

// structures:

struct EndoPt
{
	double x;
	double y;
	double z;

} point;

struct EndoLine	//really composed of two 'points'
{
	EndoPt a;
	EndoPt b;

} line;

typedef struct EndoPlane
{
	double a; 
	double b; 
	double c; 
	double d;

} plane;


static EndoPt MakePoint(double x, double y, double z)
{
	EndoPt point; 
	point.x = x;
	point.y = y;
	point.z = z;
	return point;
}

static EndoLine MakeLine(EndoPt a, EndoPt b)
{
	//l: a + t*b -> l:( x,y,z) + t(x,y,z)
	EndoLine line; 
	line.a.x = a.x;
	line.a.y = a.y;
	line.a.z = a.z;

	line.b.x = b.x;
	line.b.y = b.y;
	line.b.z = b.z;
	return line;
}

static EndoPlane MakePlane(EndoPt normal, EndoPt origin)
{
	//normal vector & origin point (can be any point on the plane)
	EndoPlane plane; 
	plane.a = normal.x; 
	plane.b = normal.y; 
	plane.c = normal.z; 
	plane.d = -(normal.x*origin.x + normal.y*origin.y + normal.z*origin.z);
	return plane; 
}

static EndoLine lineFromPoints(EndoPt p1, EndoPt p2)
{
	EndoPt b;
	b.x = (p2.x - p1.x);
	b.y = (p2.y - p1.y);
	b.x = (p2.z - p1.z);

	EndoLine line = MakeLine(p1, b);
	return line;
}

double dot(EndoPt p1, EndoPt p2)
{
	return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z); 
}

EndoPt unitVector(EndoPt pt)
{
	EndoPt result; 
	double length = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z); 
	result.x = pt.x / length; 
	result.y = pt.y / length;
	result.z = pt.z / length;
	return result; 
}

EndoPt solveIntersection(EndoPt normal, EndoPt origin, EndoLine camLine)
{
	 //laser plane normal vector, origin: point on laser plane, camline: parametric equation of the camera line
	//solve for t in parametric equation --> POI 

	EndoPt result; 
	EndoPt norm = unitVector(normal); 
	EndoPt unitCam = unitVector(camLine.b); 
	double angle = dot(norm, unitCam); 

	if (angle < 1e-6)	//line is parallel to plane
	{
		EndoPt diff = MakePoint(camLine.a.x - origin.x, camLine.a.y - origin.y, camLine.a.z - origin.z);
 		if (abs(dot(diff, norm)) < 1e-6)		//line lays on plane
			result = origin;
		else
			result = MakePoint(0.0, 0.0, 0.0);
	}

	else   //line intersects plane --> single point
	{
		double t; 
		EndoPlane laser = MakePlane(normal, origin);
		t = -(laser.d + laser.a*camLine.a.x + laser.b*camLine.a.y + laser.c*camLine.a.z)/
			(laser.a*camLine.b.x + laser.b*camLine.b.y + laser.c*camLine.b.z);

		result.x = camLine.a.x + t*camLine.b.x; 
		result.y = camLine.a.y + t*camLine.b.y;
		result.z = camLine.a.z + t*camLine.b.z;
	}

	return result; 
}

#endif // !LINALG_H