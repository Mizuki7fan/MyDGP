#pragma once
#include <QtWidgets>
#include <QWidget>
#include <QtGui>

class MyWidget :public QWidget
{
	Q_OBJECT;
public:
	MyWidget(QWidget* _parent = 0);
	~MyWidget();
protected:
	QLabel* lName;
	QPushButton* pbReturn;
protected:
signals:
	void changeWidget(int i);
};

class ControlPanel :public MyWidget
{
	Q_OBJECT;
public:
	ControlPanel(QWidget* _parent = 0);
	~ControlPanel();

private:
	QPushButton* pbGeneral;
	QPushButton* pbChap1;

};

class GeneralWidget :public MyWidget
{
	Q_OBJECT;
public:
	GeneralWidget(QWidget* _parent = 0);
	~GeneralWidget();
private:
signals:
	void PrintInfo();

private:
	QPushButton* pbPrintInfo;
};

class DiscreteDifferentialGeometryWidget :public MyWidget
{
	Q_OBJECT;
public:
	DiscreteDifferentialGeometryWidget(QWidget* _parent = 0);
	~DiscreteDifferentialGeometryWidget();
private:
signals:
	void ComputeCurvatureSignal(int,int);
private:
	QComboBox* qbLocalAverageRegion;
	QComboBox* qbCurvatureKind;
	QPushButton* pbComputeCurvature;

};