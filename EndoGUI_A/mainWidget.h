#pragma once
#ifndef __MAINWIDGET_H__
#define __MAINWIDGET_H__
#include <vtk_glew.h>

// C++ Includes
#include <vector>
#include <fstream>

// QT Includes
#include <QtGui>
#include <QTableWidget>
#include <QSpinBox>
#include <QCheckBox>
#include <QImage>
#include <QLabel>
#include <QHBoxLayout>

// VTK Includes
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkActor.h>
#include <vtkTexture.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkImageImport.h>
//#include <vtkNDITracker.h>

// Local Includes
#include "qmainwindow.h"
#include "qpushbutton.h"
#include "qlightwidget.h"
//#include "visualizationController.h"

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// stl includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// Plus Includes
#include <vtkPlusDataCollector.h>
#include "PlusConfigure.h"
#include "vtkPlusTransformRepository.h"
#include "PlusTrackedFrame.h"
#include "vtkPlusChannel.h"
#include <vtkPlusNDITracker.h>
#include <vtkPlusVolumeReconstructor.h>
#include <vtkPlusMmfVideoSource.h>
#include <vtkPlusOpenIGTLinkVideoSource.h>
#include "C:/RVTK-bin/Deps/Plus-bin/PlusApp/fCal/Toolboxes/QAbstractToolbox.h"

#include "qlightwidget.h"

// VTK forward declaration
class QVTKWidget;
class vtkRenderer;
class vtkTexture;
class vtkImageImport;
class vtkTrackerTool;

// Local forward declaration
class trackerWidget;
using namespace std;
using namespace cv;

class mainWidget : public QMainWindow
{
	Q_OBJECT

public:
	// A constructor
	mainWidget(QWidget* parent = 0);

	// A deconstructor
	~mainWidget();

private:
	// Set up GUI
	void createActions();

	// Set up GUI
	void createMenus();

	// Set up GUI 
	void createStatusBar();

	// A centralized place to create all vtk objects
	void createVTKObjects();

	// A centralized place to delete all vtk objects
	void destroyVTKObjects();

	// A Centralized place to setup all vtk pipelines
	void setupVTKPipeline();

	// Check the vtkTrackerTool flags to determine what tools are connected
	void checkToolPorts();

	PlusStatus ReadConfiguration(vtkXMLDataElement* aConfig);
	
private slots:
	// Update tracking and image data on timeout
	//void updateTrackerInfo();

	// Start tracking
	void startTrackerSlot(bool);

	// A QT slot to display information about this application
	void about();

	// A QT slot to display information about Robarts
	void aboutRobarts();

	// Create a dock window for controlling NDI tracker
	void createControlDock();

	// Create a dock window for the tracked tools information
	void createToolInformation();

	// Save the image and 4x4 matrix of tracked camer
	void collectPose();

	// View current scene from camera
	void viewScene(bool);

	// Update tracking information
	void updateTrackerInfo();

	void trackProbeSlot(bool);

	void trackCameraSlot(bool);

	void trackReferenceSlot(bool);

	void trackModelSlot(bool);

	void trackOtherSlot(bool);

	void useWebcam(bool);

	void useOvrvisionPro(bool);

	void useUltrasound(bool);

	void useEndoCam(bool);

private:
	QAction*										aboutAct;
	QAction*										quitAct;
	QAction*										aboutRobartsAct;
	QAction*										controlAct;
	QAction*										trackProbe;
	QAction*										trackCamera;
	QAction*										trackReference;
	QAction*										trackModel;
	QAction*										trackOther;
	QAction*										webcam;
	QAction*										ovrvisionPro;
	QAction*										ultrasound;
	QAction*										endoCam;
	QPushButton*									trackerButton;
	QTimer*											trackerTimer;
	QMenu*											fileMenu;
	QMenu*											helpMenu;
	QMenu*											calibMenu;
	QMenu*											controlMenu;
	QMenu*											toolMenu;
	QMenu*											cameraMenu;
	QDockWidget*									controlDock;
	QTimer*											mTimer;
	QDockWidget*									toolInfo;
	QTableWidget*									dataTable;
	QLineEdit*										stylusTipRMS;
	QVTKWidget*										qvtk;
	std::vector< QLightWidget*>						lightWidgets;
	QLabel*											videoStream;
	QHBoxLayout*									videoLayout;

	bool											isProbeVisible;
	bool											isTrackerInit;
	bool											isViewScene;
	int												imageCount = 0;
	int												tableRow = 1;

	// OpenCV matrices for images
	cv::Mat											matImage;
	cv::Mat											finalMat;

	// Plus members
	vtkSmartPointer<vtkXMLDataElement>				configRootElement = vtkSmartPointer<vtkXMLDataElement>::New();
	vtkSmartPointer<vtkPlusDataCollector>			dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();
	vtkSmartPointer<vtkPlusTransformRepository>		repository = vtkSmartPointer<vtkPlusTransformRepository>::New();

	vtkPlusDevice									*trackerDevice;
	
	// Video Devices
	vtkPlusDevice									*ovrDevice;
	vtkPlusDevice									*webcamDevice;
	vtkPlusDevice									*endoscopeDevice;
	vtkPlusDevice									*ultrasoundDevice;
	vtkPlusDevice									*endoDevice;

	// Mixers
	vtkPlusDevice									*mixerDevice;
	vtkPlusDevice									*leftMixerDevice;
	vtkPlusDevice									*rightMixerDevice;
	
	// Channels
	vtkPlusChannel									*trackerChannel;
	vtkPlusChannel									*videoChannel;
	vtkPlusChannel									*leftVideoChannel;
	vtkPlusChannel									*rightVideoChannel;
	vtkPlusChannel									*leftMixerChannel;
	vtkPlusChannel									*rightMixerChannel;
	vtkPlusChannel									*mixerChannel;

	PlusTrackedFrame								leftVideoFrame;
	PlusTrackedFrame								rightVideoFrame;
	PlusTrackedFrame								videoFrame;
	PlusTrackedFrame								leftMixerFrame;
	PlusTrackedFrame								rightMixerFrame;
	PlusTrackedFrame								mixerFrame;

	vtkPlusNDITracker								*ndiTracker;
	vtkPlusMmfVideoSource							*webcamVideo;
	vtkPlusMmfVideoSource							*endoVideo;
	vtkPlusOpenIGTLinkVideoSource					*ultrasoundVideo;

	vtkSmartPointer<vtkPlusVirtualMixer>			mixer = vtkSmartPointer<vtkPlusVirtualMixer>::New();

	// Mixers for left and right eyes
	vtkSmartPointer<vtkPlusVirtualMixer>			leftMixer = vtkSmartPointer<vtkPlusVirtualMixer>::New();
	vtkSmartPointer<vtkPlusVirtualMixer>			rightMixer = vtkSmartPointer<vtkPlusVirtualMixer>::New();

	PlusTrackedFrame								trackedFrame;

	// Plus Transform Names
	PlusTransformName								camera2TrackerName = PlusTransformName("Camera", "Tracker");
	PlusTransformName								probe2TrackerName = PlusTransformName("Probe", "Tracker");
	PlusTransformName								probe2CameraName = PlusTransformName("Probe", "Camera");
	PlusTransformName								tip2ProbeName = PlusTransformName("PointerTip", "Probe");
	PlusTransformName								tip2ImageName = PlusTransformName("PointerTip", "ImagePlane");
	PlusTransformName								tip2CameraName = PlusTransformName("PointerTip", "Camera");
	PlusTransformName								camera2ImageName = PlusTransformName("Camera", "ImagePlane");
	PlusTransformName								model2TrackerName = PlusTransformName("Model", "Tracker");
	PlusTransformName								ref2TrackerName = PlusTransformName("Reference", "Tracker");
	PlusTransformName								other2TrackerName = PlusTransformName("Other", "Tracker");

	// Plus Transforms
	vtkSmartPointer<vtkMatrix4x4>					tCamera2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTracker2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tCamera2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tProbe2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tProbe2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTip2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tTip2Probe = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tModel2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tRef2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tOther2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tCam2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkTrackerTool>					probe;

	// Transformation matrices
	vtkSmartPointer<vtkMatrix4x4>					posMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	// VTK Render Members
	vtkSmartPointer<vtkTexture>						texture = vtkSmartPointer<vtkTexture>::New();
	vtkSmartPointer<vtkRenderWindow>				renWindow = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderer>					ren = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkImageImport>					imageImport = vtkSmartPointer<vtkImageImport>::New();

	// Auxilary members
	vtkMatrix4x4									*pointerPose;

	// Input configuration file name
	std::string										inputConfigFileName;

	// Camera intrinsics file name
	std::string										intrinsicsFileName;

	// File stream for experimental output
	std::ofstream									expOutFile;

	// Subject ID
	std::string										subjectID;

	// Path to experimental results
	std::string										results_root_dir;

	// Path to calibration saved
	std::string										calibration_root_dir;

	// Calibration pose file
	std::ofstream									cam_pose_file;

	// Cam calibration file prefix and post-fix
	std::string										cam_calib_file_prefix;
	std::string										cam_calib_file_postfix;

private:
	trackerWidget*									toolTrackerWidget;

private:
	// Image counter
	int												img_index;
};

#endif // of __MAINGWIDGET_H__