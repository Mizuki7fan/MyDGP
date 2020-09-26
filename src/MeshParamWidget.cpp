#include "MeshParamWidget.h"
#include <iostream>

MeshParamWidget::MeshParamWidget(QWidget *parent)
	: QWidget(parent)
{
	wControlPanel = new ControlPanel();
	wGeneral = new GeneralWidget();
	QVBoxLayout* layout = new QVBoxLayout();
	layout->addWidget(wControlPanel);
	layout->addWidget(wGeneral);
	this->setLayout(layout);

	SetControlPanelVisible();//初始设置控制面板为可见
	connect(wControlPanel, SIGNAL(changeWidget(int)), this, SLOT(SetWidgetVisible(int)));
	connect(wGeneral, SIGNAL(ReturnToControlPanel(void)), this, SLOT(SetControlPanelVisible()));
}

MeshParamWidget::~MeshParamWidget()
{
}

void MeshParamWidget::SetControlPanelVisible()
{
	wGeneral->setVisible(false);
	wControlPanel->setVisible(true);
}


void MeshParamWidget::SetWidgetVisible(int i)
{
	wControlPanel->setVisible(false);
	switch (i)
	{
	case 0:
		wGeneral->setVisible(true);
	default:
		break;
	}

}









//	connect(pbPrintInfo, SIGNAL(clicked()), SIGNAL(PrintInfoSignal()));
//	connect(pbComputeCurvature, SIGNAL(clicked()), SIGNAL(ComputeCurvatureSignal()));