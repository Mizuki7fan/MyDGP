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

public slots:
	void SetShowWidget(int i);
private:
	QStackedWidget* m_pStackedWidget;

public:
	ControlPanel* wControlPanel;
	GeneralWidget* wGeneral;
	DiscreteDifferentialGeometryWidget* wChap1;
	Smoothing* wChap2;

};
