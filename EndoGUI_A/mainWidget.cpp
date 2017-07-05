#include <vtk_glew.h>

// Local includes
#include "mainWidget.h"
#include "trackerWidget.h"
//#include "visualizationController.h"
//#include "Control"

// C++ includes
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <future>

// QT includes
#include <QAction>
#include <QDockWidget>
#include <QFrame>
#include <QHeaderView>
#include <Qlabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QtGui>
#include <QCheckBox.h>
#include <QImage>

// VTK includes
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
//#include <vtkTrackerTool.h>
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
#include "C:/RVTK-bin/Deps/Plus-bin/PlusApp/fCal/Toolboxes/QAbstractToolbox.h"
#include "PlusMath.h"
#include "PlusXMLUtils.h"


// OpenCV 
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

mainWidget::mainWidget(QWidget* parent)
	: QMainWindow(parent)
{
	// Create the rest of the QT/GUI
	controlDock = 0;
	createActions();
	createMenus();
	createStatusBar();
	createToolInformation();

	// VTK Related objects
	createVTKObjects();
	setupVTKPipeline();
	createControlDock();
}

mainWidget::~mainWidget()
{
	this->destroyVTKObjects();		/*!< VTK cleanup			*/
	dataCollector->Stop();			/*!< Stop data collection	*/
}

void mainWidget::collectPose()
{
	trackerChannel->GetTrackedFrame(trackedFrame);
	
	string number = to_string(imageCount);

	if (ovrvisionPro->isChecked())
	{
		cv::Mat leftImage;
		cv::Mat rightImage;

		// Get frames
		leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
		rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

		// Get images
		vtkImageData *leftVtkImage = leftMixerFrame.GetImageData()->GetImage();
		vtkImageData *rightVtkImage = rightMixerFrame.GetImageData()->GetImage();

		int leftDims[3];
		int rightDims[3];
		leftVtkImage->GetDimensions(leftDims);
		rightVtkImage->GetDimensions(rightDims);

		// Copy vtkImage to cv::Mat
		leftImage = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftVtkImage->GetScalarPointer(0, 0, 0));
		rightImage = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightVtkImage->GetScalarPointer(0, 0, 0));

		cv::Mat flippedLImage;
		cv::Mat flippedRImage;
		cv::flip(leftImage, flippedLImage, 0);
		cv::flip(rightImage, flippedRImage, 0);

		string name = "./Results/pose" + number;
		string fullNameL = name + "L.png";
		string fullNameR = name + "R.png";

		// Save image
		imwrite(fullNameL, flippedLImage);
		imwrite(fullNameR, flippedRImage);
	}
	else
	{
		// Get frame
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		// Get image
		vtkImageData *image = mixerFrame.GetImageData()->GetImage();
		int dims[3];
		image->GetDimensions(dims);

		// Copy vtkImage to cv::Mat
		matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));
		
		string name = "./Results/pose" + number;
		string fullName = name + ".png";
		
		// Save image
		imwrite(fullName, matImage);
	}
	
	bool isMatrixValid(false);
	if (trackProbe->isChecked() == true)
	{
		dataTable->insertRow(tableRow);
		dataTable->setItem(tableRow, 0, new QTableWidgetItem("Probe"));

		bool isMatrixValid(false);
		if (repository->GetTransform(PlusTransformName("Probe", "Tracker"), tProbe2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			// Save data
			string nameP2T = "./Results/P2T" + number + ".csv";
			ofstream myfile(nameP2T);
			myfile << tProbe2Tracker->GetElement(0, 0) << "," << tProbe2Tracker->GetElement(0, 1) << "," << tProbe2Tracker->GetElement(0, 2) << "," << tProbe2Tracker->GetElement(0, 3)
				<< "," << tProbe2Tracker->GetElement(1, 0) << "," << tProbe2Tracker->GetElement(1, 1) << "," << tProbe2Tracker->GetElement(1, 2) << "," << tProbe2Tracker->GetElement(1, 3)
				<< "," << tProbe2Tracker->GetElement(2, 0) << "," << tProbe2Tracker->GetElement(2, 1) << "," << tProbe2Tracker->GetElement(2, 2) << "," << tProbe2Tracker->GetElement(2, 3)
				<< "," << tProbe2Tracker->GetElement(3, 0) << "," << tProbe2Tracker->GetElement(3, 1) << "," << tProbe2Tracker->GetElement(3, 2) << "," << tProbe2Tracker->GetElement(3, 3);
		
			dataTable->setItem(tableRow, 1, new QTableWidgetItem(QString::number(tProbe2Tracker->GetElement(0, 3))));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem(QString::number(tProbe2Tracker->GetElement(1, 3))));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem(QString::number(tProbe2Tracker->GetElement(2, 3))));
		}
		else
		{
			dataTable->setItem(tableRow, 1, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem("NULL"));
		}
		
		tableRow++;
	}

	if (trackCamera->isChecked() == true)
	{
		dataTable->insertRow(tableRow);
		dataTable->setItem(tableRow, 0, new QTableWidgetItem("Camera"));

		bool isMatrixValid(false);
		if (repository->GetTransform(PlusTransformName("Camera", "Tracker"), tCamera2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			// Save data
			string nameC2T = "./Results/C2T" + number + ".csv";
			ofstream myfile(nameC2T);
			myfile << tCamera2Tracker->GetElement(0, 0) << "," << tCamera2Tracker->GetElement(0, 1) << "," << tCamera2Tracker->GetElement(0, 2) << "," << tCamera2Tracker->GetElement(0, 3)
				<< "," << tCamera2Tracker->GetElement(1, 0) << "," << tCamera2Tracker->GetElement(1, 1) << "," << tCamera2Tracker->GetElement(1, 2) << "," << tCamera2Tracker->GetElement(1, 3)
				<< "," << tCamera2Tracker->GetElement(2, 0) << "," << tCamera2Tracker->GetElement(2, 1) << "," << tCamera2Tracker->GetElement(2, 2) << "," << tCamera2Tracker->GetElement(2, 3)
				<< "," << tCamera2Tracker->GetElement(3, 0) << "," << tCamera2Tracker->GetElement(3, 1) << "," << tCamera2Tracker->GetElement(3, 2) << "," << tCamera2Tracker->GetElement(3, 3);
		
			dataTable->setItem(tableRow, 1, new QTableWidgetItem(QString::number(tCamera2Tracker->GetElement(0, 3))));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem(QString::number(tCamera2Tracker->GetElement(1, 3))));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem(QString::number(tCamera2Tracker->GetElement(2, 3))));
		}
		else
		{
			dataTable->setItem(tableRow, 1, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem("NULL"));
		}
		
		tableRow++;
	}

	if (trackReference->isChecked() == true)
	{
		dataTable->insertRow(tableRow);
		dataTable->setItem(tableRow, 0, new QTableWidgetItem("Reference"));

		bool isMatrixValid(false);
		if (repository->GetTransform(PlusTransformName("Reference", "Tracker"), tRef2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			// Save data
			string nameR2T = "./Results/R2T" + number + ".csv";
			ofstream myfile(nameR2T);
			myfile << tRef2Tracker->GetElement(0, 0) << "," << tRef2Tracker->GetElement(0, 1) << "," << tRef2Tracker->GetElement(0, 2) << "," << tRef2Tracker->GetElement(0, 3)
				<< "," << tRef2Tracker->GetElement(1, 0) << "," << tRef2Tracker->GetElement(1, 1) << "," << tRef2Tracker->GetElement(1, 2) << "," << tRef2Tracker->GetElement(1, 3)
				<< "," << tRef2Tracker->GetElement(2, 0) << "," << tRef2Tracker->GetElement(2, 1) << "," << tRef2Tracker->GetElement(2, 2) << "," << tRef2Tracker->GetElement(2, 3)
				<< "," << tRef2Tracker->GetElement(3, 0) << "," << tRef2Tracker->GetElement(3, 1) << "," << tRef2Tracker->GetElement(3, 2) << "," << tRef2Tracker->GetElement(3, 3);
		
			dataTable->setItem(tableRow, 1, new QTableWidgetItem(QString::number(tRef2Tracker->GetElement(0, 3))));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem(QString::number(tRef2Tracker->GetElement(1, 3))));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem(QString::number(tRef2Tracker->GetElement(2, 3))));
		}
		else
		{
			dataTable->setItem(tableRow, 1, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem("NULL"));
		}
		
		tableRow++;
	}
	
	if (trackModel->isChecked() == true)
	{
		dataTable->insertRow(tableRow);
		dataTable->setItem(tableRow, 0, new QTableWidgetItem("Model"));

		bool isMatrixValid(false);
		if (repository->GetTransform(PlusTransformName("Model", "Tracker"), tModel2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			// Save data
			string nameM2T = "./Results/M2T" + number + ".csv";
			ofstream myfile(nameM2T);
			myfile << tModel2Tracker->GetElement(0, 0) << "," << tModel2Tracker->GetElement(0, 1) << "," << tModel2Tracker->GetElement(0, 2) << "," << tModel2Tracker->GetElement(0, 3)
				<< "," << tModel2Tracker->GetElement(1, 0) << "," << tModel2Tracker->GetElement(1, 1) << "," << tModel2Tracker->GetElement(1, 2) << "," << tModel2Tracker->GetElement(1, 3)
				<< "," << tModel2Tracker->GetElement(2, 0) << "," << tModel2Tracker->GetElement(2, 1) << "," << tModel2Tracker->GetElement(2, 2) << "," << tModel2Tracker->GetElement(2, 3)
				<< "," << tModel2Tracker->GetElement(3, 0) << "," << tModel2Tracker->GetElement(3, 1) << "," << tModel2Tracker->GetElement(3, 2) << "," << tModel2Tracker->GetElement(3, 3);
		
			dataTable->setItem(tableRow, 1, new QTableWidgetItem(QString::number(tModel2Tracker->GetElement(0, 3))));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem(QString::number(tModel2Tracker->GetElement(1, 3))));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem(QString::number(tModel2Tracker->GetElement(2, 3))));
		}
		else
		{
			dataTable->setItem(tableRow, 1, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem("NULL"));
		}
	
		tableRow++;
	}

	if (trackOther->isChecked() == true)
	{
		dataTable->insertRow(tableRow);
		dataTable->setItem(tableRow, 0, new QTableWidgetItem("Other"));

		bool isMatrixValid(false);
		if (repository->GetTransform(PlusTransformName("Other", "Tracker"), tOther2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			// Save data
			string nameO2T = "./Results/O2T" + number + ".csv";
			ofstream myfile(nameO2T);
			myfile << tOther2Tracker->GetElement(0, 0) << "," << tOther2Tracker->GetElement(0, 1) << "," << tOther2Tracker->GetElement(0, 2) << "," << tOther2Tracker->GetElement(0, 3)
				<< "," << tOther2Tracker->GetElement(1, 0) << "," << tOther2Tracker->GetElement(1, 1) << "," << tOther2Tracker->GetElement(1, 2) << "," << tOther2Tracker->GetElement(1, 3)
				<< "," << tOther2Tracker->GetElement(2, 0) << "," << tOther2Tracker->GetElement(2, 1) << "," << tOther2Tracker->GetElement(2, 2) << "," << tOther2Tracker->GetElement(2, 3)
				<< "," << tOther2Tracker->GetElement(3, 0) << "," << tOther2Tracker->GetElement(3, 1) << "," << tOther2Tracker->GetElement(3, 2) << "," << tOther2Tracker->GetElement(3, 3);
		
			dataTable->setItem(tableRow, 1, new QTableWidgetItem(QString::number(tOther2Tracker->GetElement(0, 3))));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem(QString::number(tOther2Tracker->GetElement(1, 3))));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem(QString::number(tOther2Tracker->GetElement(2, 3))));
		}
		else
		{
			dataTable->setItem(tableRow, 1, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 2, new QTableWidgetItem("NULL"));
			dataTable->setItem(tableRow, 3, new QTableWidgetItem("NULL"));
		}
		tableRow++;
	}

	imageCount++;
	toolTrackerWidget->collectPoses->setChecked(false);

}

/*!
* Centralized place to create all vtk objects
*/
void mainWidget::createVTKObjects()
{
	if (webcam->isChecked() == true)
	{
		inputConfigFileName = "./config/configWebcam.xml";
	}
	else if (ovrvisionPro->isChecked() == true)
	{
		inputConfigFileName = "./config/configOvrvisionPro.xml";
	}
	else if (ultrasound->isChecked() == true)
	{
		inputConfigFileName = "./config/configUltrasound.xml";
	}
	else if(endoCam->isChecked() == true)
	{
		inputConfigFileName = "./config/configEndoscope.xml";
	}
	else
	{
		inputConfigFileName = "./config/configTool.xml";
	}

	intrinsicsFileName = "./config/calibration.xml";

	// Create output directory
	results_root_dir = "./Results/";
	if (CreateDirectory(results_root_dir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS != GetLastError())
		std::cout << "Output directory created." << std::endl;

	// Create calibration saves directory
	calibration_root_dir = "./Calibration";
	if (CreateDirectory(calibration_root_dir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS != GetLastError())
		std::cout << "calibration directory created." << std::endl;

	trackerDevice = NULL;

	// Read configuration file
	if (PlusXmlUtils::ReadDeviceSetConfigurationFromFile(configRootElement, inputConfigFileName.c_str()) == PLUS_FAIL)
	{
		LOG_ERROR("Unable to read configuration from file" << inputConfigFileName.c_str());
		exit;
	}

	vtkPlusConfig::GetInstance()->SetDeviceSetConfigurationData(configRootElement);

	// Read configuration file
	if (dataCollector->ReadConfiguration(configRootElement) != PLUS_SUCCESS)
	{
		LOG_ERROR("Configuration incorrect for vtkPlusDataCollector.");
		exit;
	}

	isViewScene = false;
	isTrackerInit = false;
}

/*!
* A centralized place to delete all vtk objects
*/
void mainWidget::destroyVTKObjects()
{

}

/*!
* A centralized place to setup all vtk pipelines
*/
void mainWidget::setupVTKPipeline()
{
	isTrackerInit = false;
}

/*!
* A QT slot to start the QTimer to acquire tracking data
*
* If the tracker is NOT initialized, this QT slot will initialize
* the tracker by probing all the available ports.
*/
void mainWidget::startTrackerSlot(bool checked)
{
	if (checked)
	{
		//check if the tracker is initialized. If not, initialize it here
		if (!isTrackerInit)
		{
			// Get tracker
			if (dataCollector->GetDevice(trackerDevice, "TrackerDevice") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the device with ID = \"TrackerDevice\". Check config file.");
				exit;
			}

			if (ovrvisionPro->isChecked() == true)
			{

				// Get virtual mixer
				if (dataCollector->GetDevice(leftMixerDevice, "LeftTrackedVideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"LeftTrackedVideoDevice\". Check config file.");
					exit;
				}

				// Get virtual mixer
				if (dataCollector->GetDevice(rightMixerDevice, "RightTrackedVideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"RightTrackedVideoDevice\". Check config file.");
					exit;

				}

				leftMixer = dynamic_cast<vtkPlusVirtualMixer *>(leftMixerDevice);
				rightMixer = dynamic_cast<vtkPlusVirtualMixer *>(rightMixerDevice);
			}
			else if (webcam->isChecked() == true)
			{
				// Get Ovrvision Pro device
				if (dataCollector->GetDevice(webcamDevice, "VideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"VideoDevice\". Check config file.");
					exit;
				}

				// Get virtual mixer
				if (dataCollector->GetDevice(mixerDevice, "TrackedVideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"TrackedVideoDevice\". Check config file.");
					exit;
				}
				webcamVideo = dynamic_cast<vtkPlusMmfVideoSource *>(webcamDevice);
				mixer = dynamic_cast<vtkPlusVirtualMixer *>(mixerDevice);
			}
			else if (ultrasound->isChecked() == true)
			{
				// Get Ovrvision Pro device
				if (dataCollector->GetDevice(ultrasoundDevice, "USVideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"USVideoDevice\". Check config file.");
					exit;
				}

				// Get virtual mixer
				if (dataCollector->GetDevice(mixerDevice, "TrackedVideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"TrackedVideoDevice\". Check config file.");
					exit;
				}
				ultrasoundVideo = dynamic_cast<vtkPlusOpenIGTLinkVideoSource *>(ultrasoundDevice);
				mixer = dynamic_cast<vtkPlusVirtualMixer *>(mixerDevice);
			}
			else if (endoCam->isChecked() == true)
			{
				// Get Ovrvision Pro device
				if (dataCollector->GetDevice(endoDevice, "VideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"VideoDevice\". Check config file.");
					exit;
				}

				// Get virtual mixer
				if (dataCollector->GetDevice(mixerDevice, "TrackedVideoDevice") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the device with ID = \"TrackedVideoDevice\". Check config file.");
					exit;
				}
				endoVideo = dynamic_cast<vtkPlusMmfVideoSource *>(endoDevice);
				mixer = dynamic_cast<vtkPlusVirtualMixer *>(mixerDevice);
			}
			ndiTracker = dynamic_cast<vtkPlusNDITracker *>(trackerDevice);

			if (ndiTracker == NULL)
			{
				LOG_ERROR("Tracking device is not NDI/Polaris/Aurora. Could not connect.");
				exit(EXIT_FAILURE);
			}

			// Connect to devices
			std::cout << "Connecting to NDI Polaris through COM" << ndiTracker->GetSerialPort();
			if (dataCollector->Connect() != PLUS_SUCCESS)
			{
				std::cout << "....................... [FAILED]" << std::endl;
				LOG_ERROR("Failed to connect to devices!");
				exit;
			}

			if (dataCollector->Start() != PLUS_SUCCESS)
			{
				LOG_ERROR("Failed to connect to devices!");
				exit;
			}
			std::cout << "....................... [OK]" << std::endl;

			if (repository->ReadConfiguration(configRootElement) != PLUS_SUCCESS)
			{
				LOG_ERROR("Configuration incorrect for vtkPlusTransformRepository.");
				exit(EXIT_FAILURE);
			}

			if (ndiTracker->GetOutputChannelByName(trackerChannel, "TrackerStream") != PLUS_SUCCESS)
			{
				LOG_ERROR("Unable to locate the channel with Id = \"TrackerStream\". check config file.");
				exit(EXIT_FAILURE);
			}

			if (ovrvisionPro->isChecked() == true)
			{
				if (ovrDevice->GetOutputChannelByName(leftVideoChannel, "LeftVideoStream") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the channel with Id=\"OvrVideoStream\". Check config file.");
					exit(EXIT_FAILURE);
				}

				if (ovrDevice->GetOutputChannelByName(rightVideoChannel, "RightVideoStream") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the channel with Id=\"OvrVideoStream\". Check config file.");
					exit(EXIT_FAILURE);
				}

				trackerChannel->GetTrackedFrame(trackedFrame);
				leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
				rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

				// Get images
				vtkImageData *image = leftMixerFrame.GetImageData()->GetImage();
				int dims[3];
				image->GetDimensions(dims);

				// Copy vtkImage to cv::Mat
				matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));
			}
			else if(webcam->isChecked() == true)
			{
				if (webcamDevice->GetOutputChannelByName(videoChannel, "VideoStream") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the channel with Id=\"VideoStream\". Check config file.");
					exit(EXIT_FAILURE);
				}

				trackerChannel->GetTrackedFrame(trackedFrame);
				mixer->GetChannel()->GetTrackedFrame(mixerFrame);

				// Get images
				vtkImageData *image = mixerFrame.GetImageData()->GetImage();
				int dims[3];
				image->GetDimensions(dims);

				// Copy vtkImage to cv::Mat
				matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));
			}
			else if (ultrasound->isChecked() == true)
			{
				if (ultrasoundDevice->GetOutputChannelByName(videoChannel, "USVideoStream") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the channel with Id=\"USVideoStream\". Check config file.");
					exit(EXIT_FAILURE);
				}

				trackerChannel->GetTrackedFrame(trackedFrame);
				mixer->GetChannel()->GetTrackedFrame(mixerFrame);

				// Get images
				vtkImageData *image = mixerFrame.GetImageData()->GetImage();
				int dims[3];
				image->GetDimensions(dims);

				// Copy vtkImage to cv::Mat
				matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));
			}
			else if (endoCam->isChecked() == true)
			{
				if (endoDevice->GetOutputChannelByName(videoChannel, "VideoStream") != PLUS_SUCCESS)
				{
					LOG_ERROR("Unable to locate the channel with Id=\"VideoStream\". Check config file.");
					exit(EXIT_FAILURE);
				}

				trackerChannel->GetTrackedFrame(trackedFrame);
				mixer->GetChannel()->GetTrackedFrame(mixerFrame);

				// Get images
				vtkImageData *image = mixerFrame.GetImageData()->GetImage();
				int dims[3];
				image->GetDimensions(dims);

				// Copy vtkImage to cv::Mat
				matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));
			}

			isTrackerInit = true;
			toolTrackerWidget->viewSceneButton->setEnabled(true);
			toolTrackerWidget->collectPoses->setEnabled(true);

			// If tracker is initialized, start tracking
			if (isTrackerInit)
			{
				statusBar()->showMessage(tr("Tracking started."), 5000);

				checkToolPorts();

				// The VTk pipeline takes about 15 msec, so this is roughly 20 FPS
				trackerTimer->start(35);
			}
			isTrackerInit = true;
		}
		else
		{
			// Button is un-toggled
			if (isTrackerInit)
			{
				trackerTimer->stop();
				dataCollector->Stop();

				// Turn all the light widgets to blue
				for (int i = 0; i < 4; i++)
				{
					lightWidgets[i]->BlueOn();
				}
				statusBar()->showMessage(tr("stopping tracker"), 5000);
			}
		}
	}
}

void mainWidget::viewScene(bool checked)
{
	if (checked)
	{
		QImage inputImage;

		if (ovrvisionPro->isChecked() == true)
		{
			cv::Mat leftImage;
			cv::Mat rightImage;

			// Get frames
			leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
			rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

			// Get images
			vtkImageData *leftVtkImage = leftMixerFrame.GetImageData()->GetImage();
			vtkImageData *rightVtkImage = rightMixerFrame.GetImageData()->GetImage();
			
			int leftDims[3];
			int rightDims[3];
			leftVtkImage->GetDimensions(leftDims);
			rightVtkImage->GetDimensions(rightDims);

			// Copy vtkImage to cv::Mat
			leftImage = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftVtkImage->GetScalarPointer(0, 0, 0));
			rightImage = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightVtkImage->GetScalarPointer(0, 0, 0));

			cv::Mat flippedLImage;
			cv::Mat flippedRImage;
			cv::flip(leftImage, flippedLImage, 0);
			cv::flip(rightImage, flippedRImage, 0);

			cv::Size sizeL = flippedLImage.size();
			cv::Size sizeR = flippedRImage.size();
			cv::Mat finalImage(sizeL.height, sizeL.width + sizeR.width, CV_8UC3);

			cv::Mat left(finalImage, Rect(0, 0, sizeL.width, sizeL.height));
			flippedLImage.copyTo(left);

			cv::Mat right(finalImage, Rect(sizeL.width, 0, sizeR.width, sizeR.height));
			flippedRImage.copyTo(right);

			inputImage = QImage((uchar*)finalImage.data, finalImage.cols, finalImage.rows, finalImage.step, QImage::Format_RGB888);
			videoStream->setPixmap(QPixmap::fromImage(inputImage));
		}
		else
		{
			// Get frames
			mixer->GetChannel()->GetTrackedFrame(mixerFrame);

			// Get images
			vtkImageData *image = mixerFrame.GetImageData()->GetImage();

			int dims[3];
			image->GetDimensions(dims);

			// Copy vtkImage to cv::Mat
			matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));

			inputImage = QImage((uchar*)matImage.data, matImage.cols, matImage.rows, matImage.step, QImage::Format_RGB888);
			videoStream->setPixmap(QPixmap::fromImage(inputImage));
		}	
		
		if (!isTrackerInit)
		{
			trackerChannel->GetTrackedFrame(trackedFrame);

			// Get frame
			mixer->GetChannel()->GetTrackedFrame(mixerFrame);

			// Get image
			vtkImageData *image = mixerFrame.GetImageData()->GetImage();

			int dims[3];

			image->GetDimensions(dims);

			// copy vtkimage to cv::Mat
			finalMat = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));

		}
		isViewScene = true;
	}
	else
	{
		isViewScene = false;
	}
}

void mainWidget::updateTrackerInfo()
{
	if (isTrackerInit)
	{
		ren->ResetCameraClippingRange();

		// Get updated tracking information
		trackerChannel->GetTrackedFrame(trackedFrame);

		bool isProbeMatrixValid(false);
		repository->SetTransforms(trackedFrame);

		if (trackProbe->isChecked() == true)
		{
			// Check if probe is visible
			if (repository->GetTransform(probe2TrackerName, tProbe2Tracker, &isProbeMatrixValid) == PLUS_SUCCESS && isProbeMatrixValid)
			{
				toolTrackerWidget->lightWidgets[0]->GreenOn();
			}
			else
			{
				toolTrackerWidget->lightWidgets[0]->RedOn();
			}
		}
		else
		{
			toolTrackerWidget->lightWidgets[0]->BlueOn();
		}

		if (trackCamera->isChecked() == true)
		{
			// Check if cam DRB is visibile
			bool isCamMatrixValid(false);
			if (repository->GetTransform(camera2TrackerName, tCamera2Tracker, &isCamMatrixValid) == PLUS_SUCCESS && isCamMatrixValid)
			{
				toolTrackerWidget->lightWidgets[1]->GreenOn();
			}
			else
			{
				toolTrackerWidget->lightWidgets[1]->RedOn();
			}

		}
		else
		{
			toolTrackerWidget->lightWidgets[1]->BlueOn();
		}
		
		if (trackReference->isChecked() == true)
		{
			// Check if reference DRB is visible
			bool isRefMatrixValid(false);
			if (repository->GetTransform(ref2TrackerName, tRef2Tracker, &isRefMatrixValid) == PLUS_SUCCESS && isRefMatrixValid)
			{
				toolTrackerWidget->lightWidgets[2]->GreenOn();
			}
			else
			{
				toolTrackerWidget->lightWidgets[2]->RedOn();
			}
		}
		else
		{
			toolTrackerWidget->lightWidgets[2]->BlueOn();
		}

		if (trackModel->isChecked() == true)
		{
			// Check if model DRB is visible
			bool isModMatrixValid(false);
			if (repository->GetTransform(model2TrackerName, tModel2Tracker, &isModMatrixValid) == PLUS_SUCCESS && isModMatrixValid)
			{
				toolTrackerWidget->lightWidgets[3]->GreenOn();
			}
			else
			{
				toolTrackerWidget->lightWidgets[3]->RedOn();
			}
		}
		else
		{
			toolTrackerWidget->lightWidgets[3]->BlueOn();
		}

		if (trackOther->isChecked() == true)
		{
			// Check if other DRB is visible
			bool isEndoMatrixValid(false);
			if (repository->GetTransform(other2TrackerName, tOther2Tracker, &isEndoMatrixValid) == PLUS_SUCCESS && isEndoMatrixValid)
			{
				toolTrackerWidget->lightWidgets[4]->GreenOn();
			}
			else
			{
				toolTrackerWidget->lightWidgets[4]->RedOn();
			}
		}
		else
		{
			toolTrackerWidget->lightWidgets[4]->BlueOn();
		}

		if (isViewScene)
		{
			QImage inputImage;

			if (ovrvisionPro->isChecked() == true)
			{
				cv::Mat leftImage;
				cv::Mat rightImage;

				// Get frames
				leftMixer->GetChannel()->GetTrackedFrame(leftMixerFrame);
				rightMixer->GetChannel()->GetTrackedFrame(rightMixerFrame);

				// Get images
				vtkImageData *leftVtkImage = leftMixerFrame.GetImageData()->GetImage();
				vtkImageData *rightVtkImage = rightMixerFrame.GetImageData()->GetImage();

				int leftDims[3];
				int rightDims[3];
				leftVtkImage->GetDimensions(leftDims);
				rightVtkImage->GetDimensions(rightDims);

				// Copy vtkImage to cv::Mat
				leftImage = cv::Mat(leftDims[1], leftDims[0], CV_8UC3, leftVtkImage->GetScalarPointer(0, 0, 0));
				rightImage = cv::Mat(rightDims[1], rightDims[0], CV_8UC3, rightVtkImage->GetScalarPointer(0, 0, 0));

				cv::Mat flippedLImage;
				cv::Mat flippedRImage;
				cv::flip(leftImage, flippedLImage, 0);
				cv::flip(rightImage, flippedRImage, 0);

				cv::Size sizeL = flippedLImage.size();
				cv::Size sizeR = flippedRImage.size();
				cv::Mat finalImage(sizeL.height, sizeL.width + sizeR.width, CV_8UC3);

				cv::Mat left(finalImage, Rect(0, 0, sizeL.width, sizeL.height));
				flippedLImage.copyTo(left);

				cv::Mat right(finalImage, Rect(sizeL.width, 0, sizeR.width, sizeR.height));
				flippedRImage.copyTo(right);

				inputImage = QImage((uchar*)finalImage.data, finalImage.cols, finalImage.rows, finalImage.step, QImage::Format_RGB888);
				videoStream->setPixmap(QPixmap::fromImage(inputImage));
			}
			else
			{
				// Get frames
				mixer->GetChannel()->GetTrackedFrame(mixerFrame);

				// Get images
				vtkImageData *image = mixerFrame.GetImageData()->GetImage();

				int dims[3];
				image->GetDimensions(dims);

				// Copy vtkImage to cv::Mat
				matImage = cv::Mat(dims[1], dims[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));

				inputImage = QImage((uchar*)matImage.data, matImage.cols, matImage.rows, matImage.step, QImage::Format_RGB888);
				videoStream->setPixmap(QPixmap::fromImage(inputImage));
			}
		}
	}
}

void mainWidget::trackProbeSlot(bool checked)
{
	if (checked)
	{
		// Revert changes
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Probe", "Tracker"), tProbe2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Probe is now being tracked";
		}
	}
	else
	{
		tProbe2Tracker->Identity();
	}
}

void mainWidget::trackCameraSlot(bool checked)
{
	if (checked)
	{
		// Revert changes
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Camera", "Tracker"), tCamera2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Camera is now being tracked";
		}
	}
	else
	{
		tCamera2Tracker->Identity();
	}
}

void mainWidget::trackReferenceSlot(bool checked)
{
	if (checked)
	{
		// Revert changes
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Reference", "Tracker"), tRef2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Reference is now being tracked";
		}
	}
	else
	{
		tRef2Tracker->Identity();
	}
}

void mainWidget::trackModelSlot(bool checked)
{
	if (checked)
	{
		// Revert changes
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Model", "Tracker"), tModel2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Model is now being tracked";
		}
	}
	else
	{
		tModel2Tracker->Identity();
	}
}

void mainWidget::trackOtherSlot(bool checked)
{
	if (checked)
	{
		// Revert changes
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Other", "Tracker"), tOther2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Other DRB is now being tracked";
		}
	}
	else
	{
		tOther2Tracker->Identity();
	}
}

void mainWidget::useWebcam(bool checked)
{
	if (checked)
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		// Ensure that all other selections are unchecked
		ovrvisionPro->setCheckable(false);
		ultrasound->setCheckable(false);
		endoCam->setCheckable(false);

		createVTKObjects();

		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Webcam", "Tracker"), tCam2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Webcam is now being used and tracked";
		}
	}
	else
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		createVTKObjects();
		webcam->setCheckable(true);
		ovrvisionPro->setCheckable(false);
		ultrasound->setCheckable(false);
		endoCam->setCheckable(false);
	}
}

void mainWidget::useOvrvisionPro(bool checked)
{
	if (checked)
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		// Ensure that all other cameras are unselected
		webcam->setCheckable(false);
		ultrasound->setCheckable(false);
		endoCam->setCheckable(false);

		createVTKObjects();
	}
	else
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		createVTKObjects();
		webcam->setCheckable(true);
		ovrvisionPro->setCheckable(false);
		ultrasound->setCheckable(false);
		endoCam->setCheckable(false);
	}
}

void mainWidget::useUltrasound(bool checked)
{
	if (checked)
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		// Ensure that all other cameras are unselected
		webcam->setCheckable(false);
		ovrvisionPro->setCheckable(false);
		endoCam->setCheckable(false);

		createVTKObjects();

		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Ultrasound", "Tracker"), tCam2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Ultrasound is now being used and tracked";
		}
	}
	else
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		createVTKObjects();
		webcam->setCheckable(true);
		ovrvisionPro->setCheckable(false);
		ultrasound->setCheckable(false);
		endoCam->setCheckable(false);
	}
}

void mainWidget::useEndoCam(bool checked)
{
	if (checked)
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		// Ensure that all other cameras are unselected
		webcam->setCheckable(false);
		ovrvisionPro->setCheckable(false);
		ultrasound->setCheckable(false);

		createVTKObjects();

		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Endo", "Tracker"), tCam2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			cout << "Endoscope is now being used and tracked";
		}
	}
	else
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		createVTKObjects();
		webcam->setCheckable(true);
		ovrvisionPro->setCheckable(false);
		ultrasound->setCheckable(false);
		endoCam->setCheckable(false);
	}
}

void mainWidget::createActions()
{
	quitAct = new QAction(tr("&Quit"), this);
	quitAct->setShortcuts(QKeySequence::Quit);
	quitAct->setStatusTip(tr("Quit the application"));
	connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));

	aboutAct = new QAction(tr("&About App"), this);
	aboutAct->setStatusTip(tr("About this application"));
	connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

	trackProbe = new QAction(tr("Track Probe"), this);
	trackProbe->setCheckable(true);
	trackProbe->setChecked(true);
	connect(trackProbe, SIGNAL(toggled(bool)), this, SLOT(trackProbeSlot(bool)));

	trackCamera = new QAction(tr("Track Camera"), this);
	trackCamera->setCheckable(true);
	trackCamera->setChecked(true);
	connect(trackCamera, SIGNAL(toggled(bool)), this, SLOT(trackCameraSlot(bool)));

	trackReference = new QAction(tr("Track Reference"), this);
	trackReference->setCheckable(true);
	trackReference->setChecked(false);
	connect(trackReference, SIGNAL(toggled(bool)), this, SLOT(trackReferenceSlot(bool)));

	trackModel = new QAction(tr("Track Model"), this);
	trackModel->setCheckable(true);
	trackModel->setChecked(false);
	connect(trackModel, SIGNAL(toggled(bool)), this, SLOT(trackModelSlot(bool)));

	trackOther = new QAction(tr("Track Other"), this);
	trackOther->setCheckable(true);
	trackOther->setChecked(false);
	connect(trackOther, SIGNAL(toggled(bool)), this, SLOT(trackOtherSlot(bool)));

	webcam = new QAction(tr("Webcam"), this);
	webcam->setCheckable(true);
	webcam->setChecked(true);
	connect(webcam, SIGNAL(toggled(bool)), this, SLOT(useWebcam(bool)));

	ovrvisionPro = new QAction(tr("OvrvisionPro"), this);
	ovrvisionPro->setCheckable(true);
	ovrvisionPro->setChecked(false);
	connect(ovrvisionPro, SIGNAL(toggled(bool)), this, SLOT(useOvrvisionPro(bool)));

	ultrasound = new QAction(tr("Ultrasound"), this);
	ultrasound->setCheckable(true);
	ultrasound->setChecked(false);
	connect(ultrasound, SIGNAL(toggled(bool)), this, SLOT(useUltrasound(bool)));

	endoCam = new QAction(tr("Endoscope"), this);
	endoCam->setCheckable(true);
	endoCam->setChecked(false);
	connect(endoCam, SIGNAL(toggled(bool)), this, SLOT(useEndoCam(bool)));

	aboutRobartsAct = new QAction(tr("About Robarts"), this);
	aboutRobartsAct->setStatusTip(tr("About Robarts Research Insitute"));
	connect(aboutRobartsAct, SIGNAL(triggered()), this, SLOT(aboutRobarts()));
}

void mainWidget::createMenus()
{
	fileMenu = menuBar()->addMenu(tr("&File"));
	fileMenu->addSeparator();
	fileMenu->addAction(quitAct);

	toolMenu = menuBar()->addMenu(tr("&Tracked Tools"));
	toolMenu->addAction(trackProbe);
	toolMenu->addAction(trackCamera);
	toolMenu->addAction(trackReference);
	toolMenu->addAction(trackModel);
	toolMenu->addAction(trackOther);

	cameraMenu = menuBar()->addMenu(tr("&Camera Type"));
	cameraMenu->addAction(webcam);
	cameraMenu->addAction(ovrvisionPro);
	cameraMenu->addAction(ultrasound);
	cameraMenu->addAction(endoCam);

	helpMenu = menuBar()->addMenu(tr("&Help"));
	helpMenu->addSeparator();
	helpMenu->addAction(aboutAct);
	helpMenu->addAction(aboutRobartsAct);
}

void mainWidget::createToolInformation()
{
	// Create a dock widget for the tool information
	toolInfo = new QDockWidget(tr("Tool Information"), this);
	toolInfo->setAllowedAreas(Qt::RightDockWidgetArea);
	toolInfo->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
	addDockWidget(Qt::RightDockWidgetArea, toolInfo);
	toolInfo->setMinimumWidth(406);

	// Setup layout and frame
	QFrame *mainFrame = new QFrame;
	mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
	mainFrame->setLineWidth(2);

	QVBoxLayout *vlayout = new QVBoxLayout;
	vlayout->setMargin(0);
	vlayout->setSpacing(10);
	vlayout->setAlignment(Qt::AlignTop);
	mainFrame->setLayout(vlayout);

	// Create table to hold tool information
	dataTable = new QTableWidget();
	dataTable->setRowCount(1);
	dataTable->setColumnCount(4);
	dataTable->setItem(0, 0, new QTableWidgetItem("Tracked Object"));

	dataTable->setItem(0, 1, new QTableWidgetItem("x"));
	dataTable->setItem(0, 2, new QTableWidgetItem("y"));
	dataTable->setItem(0, 3, new QTableWidgetItem("z"));

	dataTable->setShowGrid(true);
	dataTable->horizontalHeader()->hide();
	dataTable->verticalHeader()->hide();

	toolInfo->setWidget(mainFrame);
	vlayout->addWidget(dataTable);
}

/*!
* Create a dock window for contrlolling NDI Tracker
*/
void mainWidget::createControlDock()
{
	if (controlDock)
	{
		controlDock->show();
	}
	else
	{
		// Create a timer here for the tracker
		trackerTimer = new QTimer(this);
		connect(trackerTimer, SIGNAL(timeout()), this, SLOT(updateTrackerInfo()));

		controlDock = new QDockWidget(tr("Tracker Control"), this);
		controlDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
		controlDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
		addDockWidget(Qt::LeftDockWidgetArea, controlDock);
		controlDock->setMinimumWidth(180);

		QFrame* mainFrame = new QFrame;
		mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
		mainFrame->setLineWidth(2);

		QGridLayout* controlsLayout = new QGridLayout;
		controlsLayout->setMargin(0);
		controlsLayout->setSpacing(10);
		controlsLayout->setAlignment(Qt::AlignTop);
		mainFrame->setLayout(controlsLayout);

		controlDock->setWidget(mainFrame);

		// Calibration widget
		QDockWidget* stylusDock = new QDockWidget(tr("Stylus tip calibration"), this);
		stylusDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
		stylusDock->setFeatures(QDockWidget::AllDockWidgetFeatures);
		stylusDock->setMinimumWidth(180);
		addDockWidget(Qt::RightDockWidgetArea, stylusDock);

		QDockWidget* videoDock = new QDockWidget(tr("Video"), this);
		videoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::TopDockWidgetArea);
		videoDock->setFeatures(QDockWidget::AllDockWidgetFeatures);
		videoDock->setMinimumWidth(180);
		addDockWidget(Qt::TopDockWidgetArea, videoDock);

		videoStream = new QLabel;
		videoDock->setWidget(videoStream);
		videoStream->show();

		QFrame* frame = new QFrame;
		frame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
		frame->setLineWidth(2);
		QVBoxLayout* vl = new QVBoxLayout;
		vl->setSpacing(10);
		vl->setAlignment(Qt::AlignTop);
		frame->setLayout(vl);
		stylusDock->setWidget(frame);

		QString tempString;
		tempString.setNum(0, 0);

		toolTrackerWidget = new trackerWidget();
		controlsLayout->addWidget(toolTrackerWidget);

		connect(toolTrackerWidget->trackerButton, SIGNAL(toggled(bool)),
			this, SLOT(startTrackerSlot(bool)));

		connect(toolTrackerWidget->collectPoses, SIGNAL(clicked()), this,
			SLOT(collectPose()));

		connect(toolTrackerWidget->viewSceneButton, SIGNAL(toggled(bool)), this,
				SLOT(viewScene(bool)));
	}
}

PlusStatus mainWidget::ReadConfiguration(vtkXMLDataElement* aConfig)
{
	LOG_TRACE("StylusCalibrationToolbox::ReadConfiguration");

	if (aConfig == NULL)
	{
		LOG_ERROR("Unable to read configuration");
		return PLUS_FAIL;
	}

	vtkXMLDataElement* fCalElement = aConfig->FindNestedElementWithName("fCal");
	if (fCalElement == NULL)
	{
		LOG_ERROR("Unable to find fCal element in XML tree!");
		return PLUS_FAIL;
	}

	return PLUS_SUCCESS;
}

void mainWidget::createStatusBar()
{
	statusBar()->showMessage(tr("Ready"), 5000);
}

/*!
* A QT slot to display information about this application
*/
void mainWidget::about()
{
	QMessageBox::about(this, tr("About Tracking GUI"),
		tr("This is a Tracking GUI \n\n"
			"By: \n\n"
			"Isabella Morgan \t\t"
			"i2morgan@uwaterloo.ca"));
}

/*!
* A QT slot to display information about Robarts Research Institute
*/
void mainWidget::aboutRobarts()
{
	QMessageBox::about(this, tr("About Robarts Research Institute"),
		tr("This program is developed at \n\n"
			"Imaging Laboratories, \n"
			"Robarts Research Institute. \n\n"
			"London, Ontario\n"
			"Canada, N6A5K8"));
}

/*!
* Check the vtkTrackerTool flags to determine
* what tools are connected
*/
void mainWidget::checkToolPorts()
{

}