#include "ControlWidget.h"
#include "qlightwidget.h"
#include "MainWindow.h"

//QT
#include <Qlabel>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout> 

ControlWidget::ControlWidget(QWidget *parent)
	:QFrame(parent)
{
	QVBoxLayout *vbox = new QVBoxLayout; 

	//Live stream of video
	streamButton = new QPushButton; 
	streamButton->setCheckable(true);
	streamButton->setText("Stream Video");
	vbox->addWidget(streamButton);

	trackerButton = new QPushButton;
	trackerButton->setCheckable(true);
	trackerButton->setText("Start Tracking");
	vbox->addWidget(trackerButton);

	mcuButton = new QPushButton;
	mcuButton->setCheckable(true);
	mcuButton->setText("Connect MCU");
	vbox->addWidget(mcuButton);

	saveButton = new QPushButton;
	saveButton->setCheckable(true);
	saveButton->setText("Save video");
	vbox->addWidget(saveButton);

	laserButton = new QPushButton;
	laserButton->setCheckable(false);
	laserButton->setText("Laser On");
	vbox->addWidget(laserButton);

	scanButton = new QPushButton; 
	scanButton->setCheckable(false);
	scanButton->setText("Start Scanning");
	vbox->addWidget(scanButton);

	QHBoxLayout *hbox = new QHBoxLayout;
	QLabel *label = new QLabel;
	QLightWidget *lw = new QLightWidget;

	label->setText(tr("Tracked Camera"));
	label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
	hbox->addWidget(label);
	hbox->addWidget(lw);
	lw->setBlue();									//uninitalized tracking
	lightWidgets.push_back(lw);
	labelWidgets.push_back(label);
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout;
	label = new QLabel;
	label->setText(tr("Tracked Laser"));
	label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
	hbox->addWidget(label);
	lw = new QLightWidget;
	hbox->addWidget(lw);
	lw->setBlue();
	lightWidgets.push_back(lw);
	labelWidgets.push_back(label);
	vbox->addLayout(hbox);

	this->setLayout(vbox);

}