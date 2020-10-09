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
	QPushButton* pbRedo;//重做
	std::vector<QFrame*> Seperator;//分割线
	int SeperatorCount = 10;//设置分割线的数目
protected:
signals:
	void changeWidget(int i);
	void Redo();
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
	QPushButton* pbChap3;
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
	void DoFairingSignal(int,int);//Laplace自乘的次数、laplace的类型
	void DoSmoothingSignal(int,int);
	void DoBilateralDenoisingSignal(double,double);
	void DoBilateralNormalFilteringSignal(double, double);

private:
	int SeperatorCount = 5;//面板中分隔符的数量
	QPushButton* pbMakeNoise;
	QPushButton* pbFairing;//做Fairing
	QPushButton* pbLaplacianSmoothing;//进行Laplacian平滑
	QPushButton* pbBilateralMeshDenoising;//进行网格的双边滤波
	QPushButton* pbBiateralNormalFiltering;//对面的法向进行滤波

	QPushButton* pbManifoldHarmonics;//频谱的角度来进行处理

	QLineEdit* leFairingPower;//Fairing的幂
	QComboBox* qbLaplacianKind;//Laplacian的种类
	QComboBox* qbIntegrationKind;//欧拉积分的类型
	QComboBox* qbLaplacianSmoothObject;//对顶点位置还是曲率进行Laplacian的平滑
};

class Parameterization :public MyWidget
{
	Q_OBJECT;
public:
	Parameterization();

private:
signals:
	void CalcTutteSignal();
	void RedoSignal();

private:
	int SeperatorCount = 1;
	QPushButton* pbCalcTutte;

};