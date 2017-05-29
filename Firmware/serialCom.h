#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QtSerialPortInfo>
#include <QString>
#include <QMessageBox>

Class serialCom : public QSerialPort{

	Q_OBJECT

public:
	serialCom();
	~serialCom();

}