#include "MainViewerWidget.h"
#include "../MeshParamWidget.h"
#include "InteractiveViewerWidget.h"
#include <QLayout>
#include <QMessageBox>
#include <iostream>

MainViewerWidget::MainViewerWidget(QWidget* _parent/* =0 */)
	:loadmeshsuccess(false)
{
	InitViewerWindow();
}

MainViewerWidget::~MainViewerWidget(void)
{
}

void MainViewerWidget::InitViewerWindow(void)
{
	CreateViewerDialog();
	CreateParamWidget(); 

	QHBoxLayout* main_layout = new QHBoxLayout();
	main_layout->addWidget(meshparamwidget,1);
	main_layout->addWidget(meshviewerwidget, 6);
	this->setLayout(main_layout);
}

void MainViewerWidget::CreateParamWidget(void)
{
	meshparamwidget = new MeshParamWidget();
	connect(meshparamwidget, SIGNAL(RedoSignal()), meshviewerwidget, SLOT(Redo()));
	connect(meshparamwidget->wControlPanel, SIGNAL(openDebug()), meshviewerwidget, SLOT(OpenDebug()));
	connect(meshparamwidget->wGeneral, SIGNAL(PrintInfo()), meshviewerwidget, SLOT(PrintMeshInfo()));
	connect(meshparamwidget->wGeneral, SIGNAL(CalcVolumeSignal()),meshviewerwidget,SLOT(CalcMeshVolume()));
	connect(meshparamwidget->wChap1, SIGNAL(ComputeCurvatureSignal(int, int)), meshviewerwidget, SLOT(ComputeCurvature(int, int)));
	connect(meshparamwidget->wChap2, SIGNAL(MakeNoiseSignal()), meshviewerwidget, SLOT(MeshMakeNoise()));
	connect(meshparamwidget->wChap2, SIGNAL(DoFairingSignal(int,int)), meshviewerwidget, SLOT(DoFairing(int,int)));
	connect(meshparamwidget->wChap2, SIGNAL(DoSmoothingSignal(int,int)), meshviewerwidget, SLOT(DoSmoothing(int,int)));
	connect(meshparamwidget->wChap2, SIGNAL(DoBilateralDenoisingSignal(double,double)), meshviewerwidget, SLOT(DoBilateralDenoising(double,double)));
	connect(meshparamwidget->wChap2, SIGNAL(DoBilateralNormalFilteringSignal(double, double)), meshviewerwidget, SLOT(DoBilateralNormalFiltering(double, double)));
	connect(meshparamwidget->wChap3, SIGNAL(CalcTutteSignal()), meshviewerwidget, SLOT(CalcTutte()));

	connect(meshviewerwidget, SIGNAL(toStateBarSignal(QString,QString)), meshparamwidget->Statebar, SLOT(StateBarSetValue(QString, QString)));

	}

void MainViewerWidget::CreateViewerDialog(void)
{
	meshviewerwidget = new InteractiveViewerWidget(NULL);
	meshviewerwidget->setAcceptDrops(true);
	connect(meshviewerwidget, SIGNAL(LoadMeshOKSignal(bool, QString)), SLOT(LoadMeshFromInner(bool, QString)));
	//connect(meshviewerwidget, SIGNAL(toStateBar(std::vector<std::string>, std::vector<std::string>)), meshparamwidget->Statebar, SLOT(StateBarSetValue(std::vector<std::string>, std::vector<std::string>)));
}

void MainViewerWidget::OpenMeshGUI(const QString & fname)
{
	if (fname.isEmpty() || !meshviewerwidget->LoadMesh(fname.toStdString()))
	{
		QString msg = "Cannot read mesh from file:\n '" + fname + "'";
		QMessageBox::critical(NULL, windowTitle(), msg);
	}
	else
	{
		loadmeshsuccess = true;
		emit(haveLoadMesh(fname));
	}
}

void MainViewerWidget::SaveMeshGUI(const QString & fname)
{
	if (fname.isEmpty() || !meshviewerwidget->SaveMesh(fname.toStdString()))
	{
		QString msg = "Cannot read mesh from file:\n '" + fname + "'";
		QMessageBox::critical(NULL, windowTitle(), msg);
	}
}

void MainViewerWidget::LoadMeshFromInner(bool OK, QString fname)
{
	loadmeshsuccess = OK;
	emit(haveLoadMesh(fname));
}

void MainViewerWidget::Open(void)
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open mesh file"),
		tr(""),
		tr("Mesh Files (*.obj *.off *.ply *.stl);;"
		"OFF Files (*.off);;"
		"OBJ Files (*.obj);;"
		"PLY Files (*.ply);;"
		"STL Files (*.stl);;"
		"All Files (*)"));
	if (!fileName.isEmpty())
	{
		OpenMeshGUI(fileName);
	}
}

void MainViewerWidget::Save(void)
{
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save mesh file"),
		tr("untitled.obj"),
		tr("OBJ Files (*.obj);;"
		"OFF Files (*.off);;"
		"PLY Files (*.ply);;"
		"STL Files (*.stl);;"
		"All Files (*)"));
	if (!fileName.isEmpty())
	{
		SaveMeshGUI(fileName);
	}
}

void MainViewerWidget::ClearMesh(void)
{
	if (loadmeshsuccess)
	{
		loadmeshsuccess = false;
		meshviewerwidget->Clear();
	}
}

void MainViewerWidget::Screenshot(void)
{
	meshviewerwidget->ScreenShot();
}

void MainViewerWidget::ShowPoints(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::POINTS);
}

void MainViewerWidget::ShowWireframe(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::WIREFRAME);
}

void MainViewerWidget::ShowHiddenLines(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::HIDDENLINES);
}

void MainViewerWidget::ShowFlatLines(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FLATLINES);
}

void MainViewerWidget::ShowFlat(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FLAT);
}

void MainViewerWidget::ShowSmooth(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::SMOOTH);
}

void MainViewerWidget::ShowCurvature(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::CURVATURE);
}

void MainViewerWidget::ShowFaceNormal(void)
{
	meshviewerwidget->SetDrawMode(InteractiveViewerWidget::FACENORMAL);
}

void MainViewerWidget::Lighting(bool b)
{
	meshviewerwidget->EnableLighting(b);
}

void MainViewerWidget::DoubleSideLighting(bool b)
{
	meshviewerwidget->EnableDoubleSide(b);
}

void MainViewerWidget::ShowBoundingBox(bool b)
{
	meshviewerwidget->SetDrawBoundingBox(b);
}

void MainViewerWidget::ShowBoundary(bool b)
{
	meshviewerwidget->SetDrawBoundary(b);
}

void MainViewerWidget::ResetView(void)
{
	meshviewerwidget->ResetView();
}

void MainViewerWidget::ViewCenter(void)
{
	meshviewerwidget->ViewCenter();
}

void MainViewerWidget::CopyRotation(void)
{
	meshviewerwidget->CopyRotation();
}

void MainViewerWidget::LoadRotation(void)
{
	meshviewerwidget->LoadRotation();
}
