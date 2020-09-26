#include "ChapterWidget.h"

ControlPanel::ControlPanel(QWidget* parent)
	: QWidget(parent)
{
	pbGeneral = new QPushButton(QStringLiteral("通用"));
	pbChap1 = new QPushButton(QStringLiteral("离散微分几何"));

	QGridLayout* pLayout = new QGridLayout();
	pLayout->addWidget(pbGeneral, 0, 0);
	pLayout->addWidget(pbChap1, 1, 0);
	pLayout->setAlignment(Qt::AlignTop);
	this->setLayout(pLayout);
	connect(pbGeneral, &QPushButton::clicked, this, [=]() {this->changeWidget(0); });
	connect(pbChap1, &QPushButton::clicked, this, [=]() {this->changeWidget(1); });
}

ControlPanel::~ControlPanel()
{
}

GeneralWidget::GeneralWidget(QWidget* parent)
	: QWidget(parent)
{
	pbReturn = new QPushButton(QStringLiteral("返回选择界面"));
	pbPrintInfo = new QPushButton(QStringLiteral("打印网格信息"));
	QGridLayout* layout = new QGridLayout();
	layout->addWidget(pbReturn, 0, 0, 1, 1);
	layout->addWidget(pbPrintInfo, 1, 0, 1, 1);
	layout->setAlignment(Qt::AlignTop);
	this->setLayout(layout);

	connect(pbReturn, &QPushButton::clicked, this, [=]() {this->ReturnToControlPanel(); });
}

GeneralWidget::~GeneralWidget()
{
}
