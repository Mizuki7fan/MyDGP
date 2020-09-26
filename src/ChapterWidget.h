#pragma once
#include <QtWidgets>
#include <QWidget>
#include <QtGui>

class ControlPanel :public QWidget
{
	Q_OBJECT;
public:
	ControlPanel(QWidget* _parent = 0);
	~ControlPanel();

private:
signals:
	void changeWidget(int i);

private:
	QPushButton* pbGeneral;
	QPushButton* pbChap1;

};

class GeneralWidget :public QWidget
{
	Q_OBJECT;
public:
	GeneralWidget(QWidget* _parent = 0);
	~GeneralWidget();
private:
signals:
	void ReturnToControlPanel();
	void PrintInfoSignal();

private:
	QPushButton* pbReturn;
	QPushButton* pbPrintInfo;
};