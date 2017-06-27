#pragma once
#ifndef ENDOMODEL_H
#define ENDOMODEL_H

#include <Vision.h>
#include <LinAlg.h>

#include <iostream>
#include <fstream>

using namespace std; 

class EndoModel
{
private:

public:

	EndoModel();
	void convertPointCloudToSurfaceMesh();
	static void savePointCloudAsPLY(string &filename); 
	static void savePointCloudAsPCD(string &filename);
	static void addPointToPointCloud(EndoPt point);
	static void savePointCloud(); 
	void saveData(EndoPt point);

	ofstream myfile; 
};

#endif // !ENDOMODEL_H
