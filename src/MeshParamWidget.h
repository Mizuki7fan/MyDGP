#pragma once

#include <QWidget>
#include <QtGui>
#include <QtWidgets>
#include <QStackedWidget>
#include "ChapterWidget.h"

class MeshParamWidget : public QWidget
{
	Q_OBJECT

public:
	MeshParamWidget(QWidget *parent = 0);
	~MeshParamWidget(void);
private:
signals:   
	void Gen_PrintInfoSignal();//General功能
	void Ch1_ComputeCurvatureSignal();//第一章
public slots:
	void SetShowWidget(int i);
private:
	QStackedWidget* m_pStackedWidget;
	QWidget* pChap1;
	
	QPushButton* pbReturn;
	QPushButton* pbGeneral;
	QPushButton* pbChap1;
public:
	ControlPanel* wControlPanel;
	GeneralWidget* wGeneral;
	DiscreteDifferentialGeometryWidget* wChap1;

};
