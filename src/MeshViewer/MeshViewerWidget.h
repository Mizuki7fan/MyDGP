#pragma once
#include <QString>
#include "QGLViewerWidget.h"
#include "../MeshDefinition/MyMesh.h"
#include <vector>

class MeshViewerWidget : public QGLViewerWidget
{
	Q_OBJECT
public:
	MeshViewerWidget(QWidget* parent = 0);
	virtual ~MeshViewerWidget(void);
	bool LoadMesh(const std::string & filename);
	void Clear(void);
	void UpdateMesh(void);
	bool SaveMesh(const std::string & filename);
	bool ScreenShot(void);
	void SetDrawBoundingBox(bool b);
	void SetDrawBoundary(bool b);
	void EnableLighting(bool b);
	void EnableDoubleSide(bool b);
	void ResetView(void);
	void ViewCenter(void);
	void CopyRotation(void);
	void LoadRotation(void);
signals:
	void LoadMeshOKSignal(bool, QString);
public slots:
	void OpenDebug(void);
	void PrintMeshInfo(void);
	void CalcMeshVolume(void);
	void ComputeCurvature(int, int);
	void MeshMakeNoise();
	void DoFairing(int);
	void DoSmoothing(int,int);

protected:
	virtual void DrawScene(void) override;
	void DrawSceneMesh(void);

private:
	void DrawPoints(void) const;
	void DrawWireframe(void) const;
	void DrawHiddenLines(void) const;
	void DrawFlatLines(void) const;
	void DrawFlat(void) const;
	void DrawSmooth(void) const;
	void DrawCurvature(void) const;
	void DrawBoundingBox(void) const;
	void DrawBoundary(void) const;
	void DrawTexture(void) const;
	void DrawColormap(void) const;
	void MapCurvature(const std::vector<double>& values);
protected:
	MyMesh* mesh;
	QString strMeshFileName;
	QString strMeshBaseName;
	QString strMeshPath;
	MyMesh::Point ptMin, ptMax;
	bool isEnableLighting;
	bool isTwoSideLighting;
	bool isDrawBoundingBox;
	bool isDrawBoundary;
	std::vector<double> colormapvalues;
	std::vector<double> curvature_v;
};
