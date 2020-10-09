#include "MeshParamWidget.h"
#include <iostream>

MeshParamWidget::MeshParamWidget(QWidget *parent)
	: QWidget(parent)
{
	wControlPanel = new ControlPanel();
	wGeneral = new GeneralWidget();
	wChap1 = new DiscreteDifferentialGeometryWidget();
	wChap2 = new Smoothing();
	wChap3 = new Parameterization();

	m_pStackedWidget = new QStackedWidget();
	m_pStackedWidget->addWidget(wControlPanel);
	m_pStackedWidget->addWidget(wGeneral);
	m_pStackedWidget->addWidget(wChap1);
	m_pStackedWidget->addWidget(wChap2);
	m_pStackedWidget->addWidget(wChap3);

	m_pStackedWidget->setCurrentIndex(0);
	QVBoxLayout* layout = new QVBoxLayout();
	layout->addWidget(m_pStackedWidget);
	layout->setMargin(0);
	this->setLayout(layout);

	connect(wControlPanel, SIGNAL(changeWidget(int)), this,SLOT(SetShowWidget(int)));
	connect(wGeneral, SIGNAL(changeWidget(int)), this, SLOT(SetShowWidget(int)));
	connect(wChap1, SIGNAL(changeWidget(int)), this, SLOT(SetShowWidget(int)));
	connect(wChap2, SIGNAL(changeWidget(int)), this, SLOT(SetShowWidget(int)));
	connect(wChap2, SIGNAL(Redo()), this, SIGNAL(RedoSignal()));
	connect(wChap3, SIGNAL(changeWidget(int)), this, SLOT(SetShowWidget(int)));
}

MeshParamWidget::~MeshParamWidget()
{
}

void MeshParamWidget::SetShowWidget(int i)
{
	m_pStackedWidget->setCurrentIndex(i);
}