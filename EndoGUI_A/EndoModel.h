#pragma once
#ifndef ENDOMODEL_H
#define ENDOMODEL_H

#include <Vision.h>
#include <LinAlg.h>

using namespace std; 

class EndoModel
{
private:

public:

	EndoModel();
	void convertPointCloudToSurfaceMesh();
	static void savePointCloudAsPLY(string &filename); 
	static void savePointCloudAsPCD(string &filename);
	void addPointToPointCloud(EndoPt);
	static void savePointCloud(); 

};

#endif // !ENDOMODEL_H
