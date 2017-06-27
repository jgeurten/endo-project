#pragma once
#ifndef LINALG_H
#define LINALG_H


// structures:

typedef struct EndoPt
{
	double x;
	double y;
	double z;

} EndoPt;

typedef struct EndoLine	//really composed of two 'points'
{
	EndoPt a;
	EndoPt b;

} EndoLine;



static EndoPt MakePoint(double x, double y, double z)
{
	EndoPt pt;
	pt.x = x;
	pt.y = y;
	pt.z = z;
	return pt;
}

static EndoLine MakeLine(EndoPt a, EndoPt b)
{
	EndoLine line;
	line.a.x = a.x;
	line.a.y = a.y;
	line.a.z = a.z;

	line.b.x = b.x;
	line.b.y = b.y;
	line.b.z = b.z;
	return line;
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

EndoPt CrossProduct(EndoPt p1, EndoPt p2)
{
	EndoPt result;
	result.x = p1.y*p2.z - (p1.z*p2.y);
	result.y = p1.x*p2.z - (p2.x*p1.z);
	result.z = p1.x*p2.y - (p2.x*p1.y);
	return result;
}

EndoPt intersectionOfLines(EndoLine l1, EndoLine l2)
{
	double t;
	EndoPt bcross, across, adelta, result;
	bcross = CrossProduct(l1.b, l2.b);
	if (bcross.x == 0 && bcross.y == 0 && bcross.z == 0)
		return MakePoint(0.0, 0.0, 0.0);

	adelta.x = l2.a.x - l1.a.x;
	adelta.y = l2.a.y - l1.a.y;
	adelta.z = l2.a.z - l1.a.z;
						   
	across = CrossProduct(adelta, l2.b);

	//may have to be range .. unsure about precision of tracking
	if (across.x / bcross.x != across.y / bcross.y || across.x / bcross.x != across.z / bcross.z)
		return MakePoint(0.0, 0.0, 0.0);

	t = across.x / bcross.x;
	result.x = l1.a.x + t*l1.b.x;
	result.y = l1.a.y + t*l1.b.y;
	result.z = l1.a.z + t*l1.b.z;
	return result;
}

#endif // !LINALG_H
