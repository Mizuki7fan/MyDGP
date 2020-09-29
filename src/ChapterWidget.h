#pragma once
#include <QtWidgets>
#include <QWidget>
#include <QtGui>

class MyWidget :public QWidget
{
	Q_OBJECT;
public:
	MyWidget();

protected:
	QLabel* lName;
	QPushButton* pbReturn;
protected:
signals:
	void changeWidget(int i);
	void openDebug();
};

class ControlPanel :public MyWidget
{
	Q_OBJECT;
public:
	ControlPanel();

private:
	QPushButton* pbDebug;
	QPushButton* pbGeneral;
	QPushButton* pbChap1;
	QPushButton* pbChap2;
};

class GeneralWidget :public MyWidget
{
	Q_OBJECT;
public:
	GeneralWidget();

private:
signals:
	void PrintInfo();
	void CalcVolumeSignal();//计算体积

private:
	QPushButton* pbPrintInfo;
	QPushButton* pbCalcVolume;
};

class DiscreteDifferentialGeometryWidget :public MyWidget
{
	Q_OBJECT;
public:
	DiscreteDifferentialGeometryWidget();

private:
signals:
	void ComputeCurvatureSignal(int,int);
private:
	QComboBox* qbLocalAverageRegion;
	QComboBox* qbCurvatureKind;
	QPushButton* pbComputeCurvature;

};

class Smoothing :public MyWidget
{
	Q_OBJECT;
public:
	Smoothing();

private:
signals:
	void MakeNoiseSignal();
	void DoFairingSignal(int);
	void DoSmoothingSignal(int,int);
private:
	QPushButton* pbMakeNoise;
	QPushButton* pbFairing;//做Fairing
	QPushButton* pbSmoothing;
	QLineEdit* leFairingPower;//Fairing的幂
	QComboBox* qbLaplacianKind;//Laplacian的种类
	QComboBox* qbIntegrationKind;//欧拉积分的类型
	QComboBox* qbSmoothObject;

};