#include "WebcamControlWidget.h"

//QT
//#include <QtWidgets>
#include <Qlabel>
//#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout> 
//#include <QSlider>

WebcamControlWidget::WebcamControlWidget(QWidget *parent)
	:QFrame(parent)
{
	QVBoxLayout *vbox = new QVBoxLayout;

	//Live stream of video
	streamButton = new QPushButton;
	streamButton->setCheckable(true);
	streamButton->setText("Stream Video");
	vbox->addWidget(streamButton);
	
	QLabel *spacer = new QLabel;
	spacer->setText("");
	vbox->addWidget(spacer);

	QLabel *titleBS = new QLabel;
	titleBS->setText("Brightness");
	titleBS->setAlignment(Qt::AlignHCenter);
	vbox->addWidget(titleBS);

	//Slider bars for contrast, brightness
	brightSlider = new QSlider;
	brightSlider->setOrientation(Qt::Horizontal);
	brightSlider->setFocusPolicy(Qt::StrongFocus);
	brightSlider->setTickPosition(QSlider::TicksBelow);
	brightSlider->setTickInterval(10);
	brightSlider->setRange(0, 100);
	vbox->addWidget(brightSlider);

	vbox->addWidget(spacer);

	QLabel *titleCS = new QLabel;
	titleCS->setText("Contrast");
	titleCS->setAlignment(Qt::AlignHCenter);
	vbox->addWidget(titleCS);

	contrastSlider = new QSlider;
	contrastSlider->setOrientation(Qt::Horizontal);
	contrastSlider->setFocusPolicy(Qt::StrongFocus);
	contrastSlider->setTickPosition(QSlider::TicksBelow);
	contrastSlider->setTickInterval(10);
	contrastSlider->setRange(0, 100);
	 
	vbox->addWidget(contrastSlider); 
	
	this->setLayout(vbox);

}