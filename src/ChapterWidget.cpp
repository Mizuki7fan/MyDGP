#include "ChapterWidget.h"

ControlPanel::ControlPanel()
{	
	pbDebug = new QPushButton(QStringLiteral("打开测试网格"));
	lName->setText(QStringLiteral("==  控制面板  =="));
	pbGeneral = new QPushButton(QStringLiteral("通用"));
	pbChap1 = new QPushButton(QStringLiteral("离散微分几何"));
	pbChap2 = new QPushButton(QStringLiteral("Smoothing"));

	QGridLayout* pLayout = new QGridLayout();
	pLayout->addWidget(pbDebug);
	pLayout->addWidget(lName);
	pLayout->addWidget(pbGeneral);
	pLayout->addWidget(pbChap1);
	pLayout->addWidget(pbChap2);

	pLayout->setAlignment(Qt::AlignTop);
	pLayout->setMargin(0);
	this->setLayout(pLayout);
	connect(pbDebug, &QPushButton::clicked, this, [=]() {this->openDebug(); });
	connect(pbGeneral, &QPushButton::clicked, this, [=]() {this->changeWidget(1); });
	connect(pbChap1, &QPushButton::clicked, this, [=]() {this->changeWidget(2); });
	connect(pbChap2, &QPushButton::clicked, this, [=]() {this->changeWidget(3); });

}

GeneralWidget::GeneralWidget()
{
	lName->setText(QStringLiteral("==  通用功能  =="));
	pbPrintInfo = new QPushButton(QStringLiteral("打印网格信息"));
	pbCalcVolume = new QPushButton(QStringLiteral("计算体积"));
	QGridLayout* layout = new QGridLayout();
	layout->addWidget(lName, 0, 0);
	layout->addWidget(pbReturn, 1, 0);
	layout->addWidget(pbPrintInfo, 2, 0);
	layout->addWidget(pbCalcVolume, 3, 0);
	layout->setAlignment(Qt::AlignTop);
	layout->setMargin(0);
	this->setLayout(layout);
	connect(pbPrintInfo, &QPushButton::clicked, this, [=]() {this->PrintInfo(); });
	connect(pbCalcVolume, SIGNAL(clicked()), SIGNAL(CalcVolumeSignal()));

}

MyWidget::MyWidget()
{
	pbReturn = new QPushButton(QStringLiteral("返回"));
	connect(pbReturn, &QPushButton::clicked, this, [=]() {
		printf("123");
		this->changeWidget(0);}
	);
	lName = new QLabel("");
	lName->setAlignment(Qt::AlignHCenter);
}


DiscreteDifferentialGeometryWidget::DiscreteDifferentialGeometryWidget()
{
	lName->setText(QStringLiteral("==  离散微分几何  =="));
	QLabel* lLocalAverageRegionKind=new QLabel(QStringLiteral("局部平均区域类型:"));
	QStringList LocalAverageRegionsKind;
	LocalAverageRegionsKind << "Barycentric" << "Voronoi" << "Mixed";
	QLabel* lCurvatureKind=new QLabel(QStringLiteral("曲率类别:"));
	QStringList CurvatureKind;
	CurvatureKind << "Mean" << "AbsoluteMean" << "Gaussian";
	qbLocalAverageRegion = new QComboBox();
	qbLocalAverageRegion->addItems(LocalAverageRegionsKind);
	qbCurvatureKind = new QComboBox();
	qbCurvatureKind->addItems(CurvatureKind);
	pbComputeCurvature = new QPushButton(QStringLiteral("计算曲率"));

	QVBoxLayout* pLayout = new QVBoxLayout();
	pLayout->addWidget(lName);
	pLayout->addWidget(pbReturn);
	pLayout->addWidget(lLocalAverageRegionKind);
	pLayout->addWidget(qbLocalAverageRegion);
	pLayout->addWidget(lCurvatureKind);
	pLayout->addWidget(qbCurvatureKind);
	pLayout->addWidget(pbComputeCurvature);

	pLayout->setAlignment(Qt::AlignTop);
	pLayout->setMargin(0);
	this->setLayout(pLayout);

	connect(pbComputeCurvature, &QPushButton::clicked, this, [=]() {
		int localaverageregion = qbLocalAverageRegion->currentIndex();
		int curvaturekind = qbCurvatureKind->currentIndex();
		this->ComputeCurvatureSignal(localaverageregion,curvaturekind); 
		});
}

Smoothing::Smoothing()
{
	lName->setText(QStringLiteral("==  SMOOTHING  =="));
	pbMakeNoise = new QPushButton(QStringLiteral("制造噪声"));
	QLabel* lFairingPower=new QLabel(QStringLiteral("Fairing的幂次:"));
	leFairingPower = new QLineEdit();
	leFairingPower->setText("1");
	pbFairing = new QPushButton(QStringLiteral("做Fairing"));
	QLabel* lLaplacianKind = new QLabel(QStringLiteral("Laplacian类型:"));
	QStringList LaplacianKind;
	LaplacianKind << "Uniform" << "Cotangent";
	qbLaplacianKind = new QComboBox();
	qbLaplacianKind->addItems(LaplacianKind);
	QFrame* line = new QFrame(), *line2 = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	line2->setFrameShape(QFrame::HLine);
	line2->setFrameShadow(QFrame::Sunken);
	QLabel* lIntegrationKind = new QLabel(QStringLiteral("欧拉积分类型:"));
	QStringList IntegrationKind;
	IntegrationKind << "implicit" << "explicit";
	qbIntegrationKind = new QComboBox();
	qbIntegrationKind->addItems(IntegrationKind);
	QLabel* lSmoothObject = new QLabel(QStringLiteral("对...进行Smooth"));
	QStringList SmoothObject;
	SmoothObject<< QStringLiteral("顶点位置") << QStringLiteral("曲率");
	qbSmoothObject=new QComboBox();
	qbSmoothObject->addItems(SmoothObject);

	pbSmoothing = new QPushButton(QStringLiteral("做Smoothing"));


	QVBoxLayout* layout = new QVBoxLayout();
	layout->addWidget(lName);
	layout->addWidget(pbReturn);
	layout->addWidget(pbMakeNoise);
	layout->addWidget(line);
	layout->addWidget(lFairingPower);
	layout->addWidget(leFairingPower);
	layout->addWidget(pbFairing);
	layout->addWidget(line2);
	layout->addWidget(lLaplacianKind);
	layout->addWidget(qbLaplacianKind);
	layout->addWidget(lIntegrationKind);
	layout->addWidget(qbIntegrationKind);
	layout->addWidget(lSmoothObject);
	layout->addWidget(qbSmoothObject);
	layout->addWidget(pbSmoothing);

	layout->setAlignment(Qt::AlignTop);
	layout->setMargin(0);
	this->setLayout(layout);

	connect(pbMakeNoise, SIGNAL(clicked()), this, SIGNAL(MakeNoiseSignal()));
	connect(pbFairing, &QPushButton::clicked, this, [=]() {
		int fairingpower = leFairingPower->text().toInt();
		this->DoFairingSignal(fairingpower);
		});
	connect(pbSmoothing, &QPushButton::clicked, this, [=]() {
		int laplacekind = qbLaplacianKind->currentIndex();
		int integrationkind = qbIntegrationKind->currentIndex();
		this->DoSmoothingSignal(laplacekind, integrationkind);
		});

}
