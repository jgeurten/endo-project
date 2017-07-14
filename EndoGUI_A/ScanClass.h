#pragma once
#ifndef SCANCLASS_H
#define SCANCLASS_H

#include <QTimer>
#include <QObject>
class ScanClass : public QObject
{
	Q_OBJECT
public:
	explicit ScanClass(bool willSleep, QString name, QObject *parent = Q_NULLPTR);
	public slots:
	void updateCount();
private:
	QTimer *timer;
	int count;
	bool m_wantToSleep;

};
#endif // !scanClass
