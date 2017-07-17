#include "ScanClass.h"
#include <QDebug>
#include <Windows.h>

ScanClass::ScanClass(bool wantToSleep, QString name, QObject *parent) :
	QObject(parent)
{
	this->setObjectName(name);
	m_wantToSleep = wantToSleep;
	count = 0;
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateCount()));
	timer->start(30);
}

void ScanClass::updateCount()
{
	Sleep(150);
	qDebug() << "slept";
}