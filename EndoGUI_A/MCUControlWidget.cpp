#include "MCUControlWidget.h"
#include "qlightwidget.h"

//QT
#include <Qlabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout> 


MCUControlWidget::MCUControlWidget(QWidget *parent)
	:QFrame(parent)
{
	QVBoxLayout *vbox = new QVBoxLayout;

	//Live stream of video
	mcuButton = new QPushButton;
	mcuButton->setCheckable(true);
	mcuButton->setText("Connect Laser");
	vbox->addWidget(mcuButton);

	QLabel *spacer = new QLabel;
	spacer->setText("");
	vbox->addWidget(spacer);

	laserButton = new QPushButton;
	laserButton->setCheckable(true);
	laserButton->setText("Toggle Laser");
	vbox->addWidget(laserButton);

	this->setLayout(vbox);
}