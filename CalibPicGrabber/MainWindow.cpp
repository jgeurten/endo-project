#include "MainWindow.h"

//OpenCv includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>

//QT includes
#include <QtWidgets>
#include <QThread>
#include <QPainter>
#include <QMainWindow>
#include <QDesktopWidget>
#include <qmessagebox.h>
#include <qfile>
#include <qstring.h>
#include <QAction>
#include <QMenu>
#include <QStatusBar>
#include <QPushButton>
#include <qabstractbutton.h>
#include <QStyle>
#include <qfiledialog.h>
#include <qmediaplayer.h>
#include <qcolor.h>
#include <qdir.h>
#include <qlineedit.h>
#include <qimage.h>
#include <qslider.h>
#include <qpaintdevice.h>
#include <qcamera.h>
#include <qcamerainfo.h>

//MSDN 
/*#include <ks.h>
#include <ksmedia.h>	//ksproperty - focus
#include <ksproxy.h>
#include <wdmguid.h>*/
#include <dshow.h>

MainWindow::MainWindow(QWidget *parent)
	:QMainWindow(parent)
{
	createVideoWidget();
	intitCamera();
	createTimer();

	createStatusBar();
	resize(QDesktopWidget().availableGeometry(this).size()*0.6);
}



MainWindow::~MainWindow()
{
	if (capture.isOpened())
		capture.release();
}

void MainWindow::createVideoWidget()
{
	QWidget *videoWidget = new QWidget(this);
	setCentralWidget(videoWidget);

	capture_button = new QPushButton(videoWidget);
	capture_button->setText(tr("Capture"));
	capture_button->setGeometry(QRect(50, 200, 100, 40));
	connect(capture_button, SIGNAL(clicked()), this, SLOT(capture_button_pressed()));

	/*
	brightSlider = new QSlider(this);
	contrastSlider = new QSlider(this);
	brightSlider->setMinimum(0);
	contrastSlider->setMinimum(0);
	brightSlider->setMaximum(30);
	contrastSlider->setMaximum(30);
	brightSlider->  setTickInterval(1);
	contrastSlider->setTickInterval(1);

	connect(brightSlider, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
	connect(contrastSlider, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
	*/
}

void MainWindow::createTimer()
{
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateFeed()));
	timer->start(34);
}

void MainWindow::intitCamera()
{
	capture = cv::VideoCapture(0);
	if (capture.isOpened())
	{
		capture.set(CV_CAP_PROP_AUTOFOCUS, 0);	//disable autofocus
		capture.set(CV_CAP_PROP_FPS, 30);

		//setCameraControl();
	}


}

void MainWindow::createStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}


void MainWindow::capture_button_pressed()
{
	cv::Mat picture;
	capture >> picture;

	QString filename = QFileDialog::getSaveFileName(this, tr("Save File"),
		"C:/Users/jgeurten/Documents/endo-project/endo-project/CameraCalib/captures/R",
		tr("Image Files (*.png *.xpm *.jpg"));

	cv::imwrite(filename.toStdString(), picture);

}

void MainWindow::updateFeed()
{
	if (capture.isOpened())
	{
		cv::namedWindow("Control", CV_WINDOW_NORMAL);
		cvCreateTrackbar("Brightness", "Control", &brightness, 100);
		cvCreateTrackbar("Contrast", "Control", &contrast, 100);
		cvCreateTrackbar("Exposure", "Control", &exposure, 100);

		//cvCreateTrackbar("Focus", "Control", &focus, 1000);

		capture.set(CV_CAP_PROP_CONTRAST, (double)contrast);
		capture.set(CV_CAP_PROP_BRIGHTNESS, (double)brightness);
		capture.set(CV_CAP_PROP_EXPOSURE, (double)exposure);

		capture >> feedImg;
		cv::Size s = feedImg.size();
		image = QImage((const unsigned char*)(feedImg.data), feedImg.cols, feedImg.rows, feedImg.cols*feedImg.channels(), QImage::Format_RGB888).rgbSwapped();
		repaint();
	}


}


void MainWindow::setCameraControl()
{
	QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
	camera = new QCamera(QCameraInfo::defaultCamera());

	QCameraFocus *focus = camera->focus();

	focus->setFocusPointMode(QCameraFocus::FocusPointCustom);
	focus->zoomTo(6.0, 6.0);
	
}

void MainWindow::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.drawImage(QRectF(300, 100, 640, 480), image);
	qDebug() << "Stream Video Thread";
}
