#include "WebcamControlWidget.h"

//QT
//#include <QtWidgets>
//#include <Qlabel>
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

	//Slider bars for contrast, brightness
	brightSlider = new QSlider(Qt::Vertical); 
	brightSlider->setFocusPolicy(Qt::StrongFocus);
	brightSlider->setTickPosition(QSlider::TicksBothSides);
	brightSlider->setTickInterval(1);
	brightSlider->setMaximum(100);
	brightSlider->setMinimum(0);
	vbox->addWidget(brightSlider);
	
	contrastSlider = new QSlider(Qt::Vertical);
	contrastSlider->setFocusPolicy(Qt::StrongFocus);
	contrastSlider->setTickPosition(QSlider::TicksBothSides);
	contrastSlider->setTickInterval(1);
	contrastSlider->setMaximum(100);
	contrastSlider->setMinimum(0);
	vbox->addWidget(contrastSlider); 
	
	this->setLayout(vbox);

}