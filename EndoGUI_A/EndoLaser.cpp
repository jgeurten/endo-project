//Local includes
#include "EndoLaser.h"
#include "MainWindow.h"
#include "Vision.h"

// VTK includes
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkTrackerTool.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>

// Plus
#include <vtkPlusNDITracker.h>
#include "PlusTrackedFrame.h"
#include "PlusConfigure.h"
#include "vtkCommand.h"
#include "vtkCallbackCommand.h"
#include "vtkPlusDataCollector.h"
#include "vtkPlusChannel.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusDevice.h"
#include "vtkPlusRfProcessor.h"
#include "vtkPlusSavedDataSource.h"
#include "vtkPlusVirtualMixer.h"
#include "I:/d/RVTK7.1-bin/Deps/Plus-bin/PlusApp/fCal/Toolboxes/QAbstractToolbox.h"
#include "PlusMath.h"
#include "PlusXMLUtils.h"

EndoLaser::EndoLaser() {

	vector<cv::Point3d> _position, _rotation; 
	for (int = 0; i < 3; i++)			//initalize arbitrary non-zero poisiton/rotation in 3 space
	{
		_position.push_back(1.0);
		_rotation.push_back(1.0);
	}
	this.position = _position; 
	this.rotation = _rotation; 
}

void EndoLaser::collectPose()
{
	bool isMatrixValid(false);
	if (repository->GetTransform(PlusTransformName("Laser", "Tracker"), tLaser2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
	{
		// Save data
		myfile << tLaser2Tracker->GetElement(0, 0) << "," << tLaser2Tracker->GetElement(0, 1) << "," << tLaser2Tracker->GetElement(0, 2) << "," << tLaser2Tracker->GetElement(0, 3)
			<< "," << tLaser2Tracker->GetElement(1, 0) << "," << tLaser2Tracker->GetElement(1, 1) << "," << tLaser2Tracker->GetElement(1, 2) << "," << tLaser2Tracker->GetElement(1, 3)
			<< "," << tLaser2Tracker->GetElement(2, 0) << "," << tLaser2Tracker->GetElement(2, 1) << "," << tLaser2Tracker->GetElement(2, 2) << "," << tLaser2Tracker->GetElement(2, 3)
			<< "," << tLaser2Tracker->GetElement(3, 0) << "," << tLaser2Tracker->GetElement(3, 1) << "," << tLaser2Tracker->GetElement(3, 2) << "," << tLaser2Tracker->GetElement(3, 3);

		position.push_back(GetElement(0, 3));
		position.push_back(GetElement(1, 3));
		position.push_back(GetElement(2, 3));

		rotation.push_back(GetElement(0, 0));
		rotation.push_back(GetElement(1, 0));
		rotation.push_back(GetElement(2, 0));
	}
	else
	{
		position.push_back(0.0);
		position.push_back(0.0);
		position.push_back(0.0);
	}
}

vector<cv::Point3d> EndoLaser::getPosition()
{
	return this.position; 
}

vector<cv::Point3d> EndoLaser::getRotation()
{
	return this.rotation;
}