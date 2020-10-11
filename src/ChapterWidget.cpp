#include "ChapterWidget.h"

ControlPanel::ControlPanel()
{	
	pbDebug = new QPushButton(QStringLiteral("打开测试网格"));

	lName->setText(QStringLiteral("==  控制面板  =="));
	pbGeneral = new QPushButton(QStringLiteral("通用"));
	pbChap1 = new QPushButton(QStringLiteral("离散微分几何"));
	pbChap2 = new QPushButton(QStringLiteral("Smoothing"));
	pbChap3 = new QPushButton(QStringLiteral("参数化"));

	QGridLayout* pLayout = new QGridLayout();
	pLayout->addWidget(pbDebug);
	pLayout->addWidget(pbRedo);
	pLayout->addWidget(lName);
	pLayout->addWidget(pbGeneral);
	pLayout->addWidget(pbChap1);
	pLayout->addWidget(pbChap2);
	pLayout->addWidget(pbChap3);
	
	pLayout->setAlignment(Qt::AlignTop);
	pLayout->setMargin(0);
	this->setLayout(pLayout);
	connect(pbDebug, &QPushButton::clicked, this, [=]() {this->openDebug(); });
	connect(pbGeneral, &QPushButton::clicked, this, [=]() {this->changeWidget(1); });
	connect(pbChap1, &QPushButton::clicked, this, [=]() {this->changeWidget(2); });
	connect(pbChap2, &QPushButton::clicked, this, [=]() {this->changeWidget(3); });
	connect(pbChap3, &QPushButton::clicked, this, [=]() {this->changeWidget(4); });

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
	pbRedo = new QPushButton(QStringLiteral("重做"));
	Seperator.resize(SeperatorCount);
	for (int iter = 0; iter < SeperatorCount; iter++)
	{
		Seperator[iter] = new QFrame();
		Seperator[iter]->setFrameShape(QFrame::HLine);
		Seperator[iter]->setFrameShadow(QFrame::Sunken);
	}
	connect(pbReturn, &QPushButton::clicked, this, [=]() {
		this->changeWidget(0);}
	);
	connect(pbRedo, &QPushButton::clicked, this, [=]() {
		this->Redo(); });
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

	QLabel* lLaplacianKind = new QLabel(QStringLiteral("Laplacian类型:"));
	QStringList LaplacianKind;
	LaplacianKind << "Uniform" << "Cotangent";
	qbLaplacianKind = new QComboBox();
	qbLaplacianKind->addItems(LaplacianKind);
	
	QLabel* lFairingPower=new QLabel(QStringLiteral("Fairing的幂次:"));
	leFairingPower = new QLineEdit();
	leFairingPower->setText("1");
	leFairingPower->setEnabled(false);
	
	pbFairing = new QPushButton(QStringLiteral("做Fairing"));
	QLabel* lIntegrationKind = new QLabel(QStringLiteral("欧拉积分类型:"));
	QStringList IntegrationKind;
	IntegrationKind << "Explicit" << "Implicit";
	qbIntegrationKind = new QComboBox();
	qbIntegrationKind->addItems(IntegrationKind);
	
	QLabel* lLaplacianSmoothObject = new QLabel(QStringLiteral("对...进行Smooth"));
	QStringList LaplacianSmoothObject;
	LaplacianSmoothObject<< QStringLiteral("顶点位置") << QStringLiteral("曲率");
	qbLaplacianSmoothObject =new QComboBox();
	qbLaplacianSmoothObject->addItems(LaplacianSmoothObject);
	qbLaplacianSmoothObject->setEnabled(false);

	QLabel* lDenoisingStds = new QLabel(QStringLiteral("标准差σ_s:"));
	QDoubleSpinBox* sbDenoisingStds = new QDoubleSpinBox();
	sbDenoisingStds->setMinimum(0.01);
	sbDenoisingStds->setMaximum(10);
	sbDenoisingStds->setSingleStep(0.1);//设置步长
	sbDenoisingStds->setValue(1);

	QLabel* lDenoisingStdr = new QLabel(QStringLiteral("标准差σ_r:"));
	QDoubleSpinBox* sbDenoisingStdr = new QDoubleSpinBox();
	sbDenoisingStdr->setMinimum(0.01);
	sbDenoisingStdr->setMaximum(10);
	sbDenoisingStdr->setSingleStep(0.1);//设置步长
	sbDenoisingStdr->setValue(1);

	int seperator_idx = 0;
	int line_count = 0;

	pbLaplacianSmoothing = new QPushButton(QStringLiteral("Laplacian Smoothing"));
	pbBilateralMeshDenoising = new QPushButton(QStringLiteral("Bilateral Mesh Denoising"));
	pbBiateralNormalFiltering = new QPushButton(QStringLiteral("Bilateral Normal Filtering"));
	pbManifoldHarmonics = new QPushButton(QStringLiteral("Manifold Harmonics"));

	QGridLayout* layout = new QGridLayout();
	layout->addWidget(lName, line_count, 0, 1, 2); line_count++;
	layout->addWidget(pbReturn, line_count,0,1,2); line_count++;
	layout->addWidget(pbRedo, line_count, 0, 1, 2); line_count++;
	layout->addWidget(pbMakeNoise, line_count,0,1,2); line_count++;
	layout->addWidget(lLaplacianKind, line_count,0,1,1);
	layout->addWidget(qbLaplacianKind, line_count,1,1,1); line_count++;
	layout->addWidget(Seperator[seperator_idx], line_count,0,1,2); seperator_idx++; line_count++;
	layout->addWidget(lFairingPower, line_count,0,1,1);
	layout->addWidget(leFairingPower, line_count,1,1,1); line_count++;
	layout->addWidget(pbFairing, line_count,0,1,2); line_count++;
	layout->addWidget(Seperator[seperator_idx], line_count,0,1,2); seperator_idx++; line_count++;
	layout->addWidget(lIntegrationKind, line_count,0,1,1);
	layout->addWidget(qbIntegrationKind, line_count,1,1,1); line_count++;
	layout->addWidget(lLaplacianSmoothObject, line_count,0,1,1);
	layout->addWidget(qbLaplacianSmoothObject, line_count,1,1,1); line_count++;
	layout->addWidget(pbLaplacianSmoothing, line_count,0,1,2); line_count++;
	layout->addWidget(Seperator[seperator_idx], line_count,0,1,2); seperator_idx++; line_count++;
	layout->addWidget(lDenoisingStds, line_count, 0, 1, 1);
	layout->addWidget(sbDenoisingStds, line_count, 1, 1, 1); line_count++;
	layout->addWidget(lDenoisingStdr, line_count, 0, 1, 1);
	layout->addWidget(sbDenoisingStdr, line_count, 1, 1, 1); line_count++;
	layout->addWidget(pbBilateralMeshDenoising, line_count, 0, 1, 2); line_count++;
	layout->addWidget(pbBiateralNormalFiltering, line_count, 0, 1, 2); line_count++;
	layout->addWidget(Seperator[seperator_idx], line_count, 0, 1, 2); seperator_idx++; line_count++;
	layout->addWidget(pbManifoldHarmonics, line_count, 0, 1, 2); line_count++;

	layout->setAlignment(Qt::AlignTop);
	layout->setMargin(0);
	this->setLayout(layout);

	connect(pbMakeNoise, SIGNAL(clicked()), this, SIGNAL(MakeNoiseSignal()));
	connect(pbFairing, &QPushButton::clicked, this, [=]() {
		int fairingpower = leFairingPower->text().toInt();
		int lapkind = qbLaplacianKind->currentIndex();
		this->DoFairingSignal(fairingpower,lapkind);
		});
	connect(pbLaplacianSmoothing, &QPushButton::clicked, this, [=]() {
		int laplacekind = qbLaplacianKind->currentIndex();
		int integrationkind = qbIntegrationKind->currentIndex();
		this->DoSmoothingSignal(laplacekind, integrationkind);
		});
	connect(pbBilateralMeshDenoising, &QPushButton::clicked, this, [=]() {
		double stdevs = sbDenoisingStds->value();
		double stdevr = sbDenoisingStdr->value();
		this->DoBilateralDenoisingSignal(stdevs,stdevr);
		});
	connect(pbBiateralNormalFiltering, &QPushButton::clicked, this, [=]() {
		double stdevs = sbDenoisingStds->value();
		double stdevr = sbDenoisingStdr->value();
		this->DoBilateralNormalFilteringSignal(stdevs, stdevr);
		});
}

Parameterization::Parameterization()
{
	lName->setText(QStringLiteral("==  参数化  =="));
	pbCalcTutte = new QPushButton(QStringLiteral("Tutte"));
	pbCalcLSCM = new QPushButton(QStringLiteral("LSCM"));
	int seperator_idx = 0;
	int line_count = 0;

	QGridLayout* layout = new QGridLayout();
	layout->addWidget(lName, line_count, 0, 1, 2); line_count++;
	layout->addWidget(pbReturn, line_count, 0, 1, 2); line_count++;
	layout->addWidget(Seperator[seperator_idx], line_count, 0, 1, 2); seperator_idx++; line_count++;
	layout->addWidget(pbCalcTutte, line_count, 0, 1, 2); line_count++;
	layout->addWidget(pbCalcLSCM, line_count, 0, 1, 2); line_count++;
	layout->setAlignment(Qt::AlignTop);
	layout->setMargin(0);
	this->setLayout(layout);
	connect(pbCalcTutte, &QPushButton::clicked, this, [=]() {this->CalcTutteSignal();});
	connect(pbCalcLSCM, &QPushButton::clicked, this, [=]() {this->CalcLSCMSignal(); });
}

StateBar::StateBar(int num)
{
	count = num;
	QFrame* Seperator = new QFrame();
	Seperator->setFrameShape(QFrame::HLine);
	Seperator->setFrameShadow(QFrame::Raised);

	Item.resize(num); Value.resize(num);
	for (int i = 0; i < num; i++)
	{
		Item[i] = new QLabel();
		Value[i] = new QLabel();
	}

	int line_count = 0;
	QGridLayout* layout = new QGridLayout();
	layout->addWidget(Seperator, line_count, 0, 1, 2); line_count++;
	for (int i = 0; i < num; i++)
	{
		layout->addWidget(Item[i], line_count, 0, 1, 1);
		layout->addWidget(Value[i], line_count, 1, 1, 1);
		line_count++;
	}
	this->setLayout(layout);
}

void StateBar::StateBarSetValue(QString item, QString value)
{
	int N = Item.size() - 1;
	for (int i=0;i<N;i++)
	{
		Item[N-i]->setText(Item[N-i-1]->text());
		Value[N-i]->setText(Value[N-i-1]->text());
	}
	Item[0]->setText(item);
	Value[0]->setText(value);
}
