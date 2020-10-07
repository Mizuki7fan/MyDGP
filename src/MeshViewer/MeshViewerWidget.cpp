#include <QtCore>
//#include <OpenMesh/Core/IO/MeshIO.hh>
#include "MeshViewerWidget.h"
#include <iostream>
#ifdef MESHTOOL_USE_OPENMESH
#include "../MeshDefinition/MyMesh_Openmesh.h"
#endif
//#include <QOpenGLTexture>
//#include <QOpenGLFrameBufferObject>
#include "../DGPAlgorithm.h"
#include <map>
#include <QMessageBox>

MeshViewerWidget::MeshViewerWidget(QWidget* parent)
	: QGLViewerWidget(parent),
	ptMin(0.0,0.0,0.0),
	ptMax(0.0, 0.0, 0.0),
	isEnableLighting(true),
	isTwoSideLighting(false),
	isDrawBoundingBox(false),
	isDrawBoundary(false)
{
	mesh = new Mesh();
}

MeshViewerWidget::~MeshViewerWidget(void)
{
}

bool MeshViewerWidget::LoadMesh(const std::string & filename)
{
	Clear();
	bool read_OK = mesh->Load(filename);
	std::cout << "Load mesh from file " << filename << std::endl;
	if (read_OK)
	{
		strMeshFileName = QString::fromStdString(filename);
		QFileInfo fi(strMeshFileName);
		strMeshPath = fi.path();
		strMeshBaseName = fi.baseName();
		UpdateMesh();
		update();
		return true;
	}
	return false;
}

void MeshViewerWidget::Clear(void)
{
	mesh->Clear();
}

void MeshViewerWidget::UpdateMesh(void)
{
	mesh->UpdateNormals();
	if (mesh->VerticesEmpty())
	{
		std::cerr << "ERROR: UpdateMesh() No vertices!" << std::endl;
		return;
	}
	ptMin[0] = ptMin[1] = ptMin[2] = DBL_MAX;
	ptMax[0] = ptMax[1] = ptMax[2] = -DBL_MAX;

	for (int i = 0; i < mesh->NVertices(); i++)
	{
		Eigen::Vector3d p = mesh->getPoint(i);
		if (p.x() < ptMin.x()) ptMin.x() = p.x();
		if (p.y() < ptMin.y()) ptMin.y() = p.y();
		if (p.z() < ptMin.z()) ptMin.z() = p.z();
		if (p.x() > ptMax.x()) ptMax.x() = p.x();
		if (p.y() > ptMax.y()) ptMax.y() = p.y();
		if (p.z() > ptMax.z()) ptMax.z() = p.z();
	}

	double avelen = 0.0;
	double maxlen = 0.0;
	double minlen = DBL_MAX;
	for (int i=0;i<mesh->NEdges();i++)
	{
		double len = mesh->CalcEdgeLength(i);
		maxlen = len > maxlen ? len : maxlen;
		minlen = len < minlen ? len : minlen;
		avelen += len;
	}

	SetScenePosition((ptMin + ptMax) * 0.5, (ptMin - ptMax).norm() * 0.5);
	std::cout << "Information of the input mesh:" << std::endl;
	std::cout << "  [V, E, F] = [" << mesh->NVertices() << ", " << mesh->NEdges() << ", " << mesh->NFaces() << "]\n";
	std::cout << "  BoundingBox:\n";
	std::cout << "  X: [" << ptMin[0] << ", " << ptMax[0] << "]\n";
	std::cout << "  Y: [" << ptMin[1] << ", " << ptMax[1] << "]\n";
	std::cout << "  Z: [" << ptMin[2] << ", " << ptMax[2] << "]\n";
	std::cout << "  Diag length of BBox: " << (ptMax - ptMin).norm() << std::endl;
	std::cout << "  Edge Length: [" << minlen << ", " << maxlen << "]; AVG: " << avelen / mesh->NEdges() << std::endl;
}

bool MeshViewerWidget::SaveMesh(const std::string & filename)
{
	return mesh->Write(filename);
}

bool MeshViewerWidget::ScreenShot()
{
	update();
	QString filename = strMeshPath + "/" + QDateTime::currentDateTime().toString("yyyyMMddHHmmsszzz") + QString(".png");
	QImage image = grabFramebuffer();
	image.save(filename);
	std::cout << "Save screen shot to " << filename.toStdString() << std::endl;
	return true;
}

void MeshViewerWidget::SetDrawBoundingBox(bool b)
{
	isDrawBoundingBox = b;
	update();
}
void MeshViewerWidget::SetDrawBoundary(bool b)
{
	isDrawBoundary = b;
	update();
}
void MeshViewerWidget::EnableLighting(bool b)
{
	isEnableLighting = b;
	update();
}
void MeshViewerWidget::EnableDoubleSide(bool b)
{
	isTwoSideLighting = b;
	update();
}

void MeshViewerWidget::ResetView(void)
{
	ResetModelviewMatrix();
	ViewCenter();
	update();
}

void MeshViewerWidget::ViewCenter(void)
{
	if (!mesh->VerticesEmpty())
	{
		UpdateMesh();
	}
	update();
}

void MeshViewerWidget::CopyRotation(void)
{
	CopyModelViewMatrix();
}

void MeshViewerWidget::LoadRotation(void)
{
	LoadCopyModelViewMatrix();
	update();
}

void MeshViewerWidget::OpenDebug(void)
{
	Clear();
	LoadMesh("D:/repository/GeometricProcessing/Housework/src/example/alien_remesh.obj");
}

void MeshViewerWidget::PrintMeshInfo(void)
{
	std::cout << "Mesh Info:\n";
	std::cout << "  [V, E, F] = [" << mesh->NVertices() << ", " << mesh->NEdges() << ", " << mesh->NFaces() << "]\n";
	std::cout << "  BoundingBox:\n";
	std::cout << "  X: [" << ptMin[0] << ", " << ptMax[0] << "]\n";
	std::cout << "  Y: [" << ptMin[1] << ", " << ptMax[1] << "]\n";
	std::cout << "  Z: [" << ptMin[2] << ", " << ptMax[2] << "]\n";
	std::cout << "  Diag length of BBox: " << (ptMax - ptMin).norm() << std::endl;
}

void MeshViewerWidget::CalcMeshVolume(void)
{
	mesh->ComputeMeshVolume();
}

void MeshViewerWidget::ComputeCurvature(int i, int j)
{
	std::vector<double> value;
	DGPAlgorithm::ComputeCurvature(i,j,*mesh, value);
	MapCurvature(value);
}

void MeshViewerWidget::MeshMakeNoise()
{
	DGPAlgorithm::MakeNoise(*mesh);
}

void MeshViewerWidget::DoFairing(int lap_power, int lap_kind)
{
	if (lap_power <=0)
	{
		QMessageBox::critical(NULL, windowTitle(), QStringLiteral("´íÎóµÄÃÝÖµ"));
		return;
	}
	DGPAlgorithm::DoFairing(*mesh, lap_power,lap_kind);

}

void MeshViewerWidget::DoSmoothing(int laplacekind,int integrationkind)
{
	DGPAlgorithm::DoSmoothing(*mesh,laplacekind,integrationkind);
}

void MeshViewerWidget::DoBilateralDenoising(double stdevs,double stdevr)
{
	DGPAlgorithm::DoBilateralDenoising(*mesh, stdevs,stdevr);

}

void MeshViewerWidget::DoBilateralNormalFiltering(double stdevs, double stdevr)
{
	DGPAlgorithm::DoBilateralNormalFiltering(*mesh, stdevs, stdevr);
}

void MeshViewerWidget::DrawScene(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(&projectionmatrix[0]);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(&modelviewmatrix[0]);
	//DrawAxis();
	if (isDrawBoundingBox) DrawBoundingBox();
	if (isDrawBoundary) DrawBoundary();
	if (isEnableLighting) glEnable(GL_LIGHTING);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, isTwoSideLighting);
	DrawSceneMesh();
	if (isEnableLighting) glDisable(GL_LIGHTING);
}

void MeshViewerWidget::DrawSceneMesh(void)
{
	if (mesh->NVertices() == 0) { return; }
	SetMaterial();
	switch (drawmode)
	{
	case POINTS:
		DrawPoints();
		break;
	case WIREFRAME:
		DrawWireframe();
		break;
	case HIDDENLINES:
		DrawHiddenLines();
		break;
	case FLATLINES:
		DrawFlatLines();
		break;
	case FLAT:
		glColor3d(0.8, 0.8, 0.8);
		DrawFlat();
		break;
	case SMOOTH:
		DrawSmooth();
		break;
	case CURVATURE:
		DrawCurvature();
		break;
	case FACENORMAL:
		DrawFaceNormal();
		break;
	default:
		break;
	}
}

void MeshViewerWidget::DrawPoints(void) const
{
	glColor3d(1.0, 0.5, 0.5);
	glPointSize(5);
	glBegin(GL_POINTS);
	for ( int i = 0; i < mesh->NVertices(); i++)
	{
			glVertex3d(mesh->getPoint(i)[0], mesh->getPoint(i)[1], mesh->getPoint(i)[2]);
			Eigen::Vector3d n = mesh->getVertexNormal(i);
			glNormal3d(n.x(), n.y(), n.z());
	}
	glEnd();
}

void MeshViewerWidget::DrawWireframe(void) const
{
	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_LINES);
	for (int i=0;i<mesh->NEdges();i++)
	{
		int v1idx, v2idx;
		mesh->getEdgeVertices(i, v1idx, v2idx);
		glNormal3d(mesh->getVertexNormal(v1idx).x(), mesh->getVertexNormal(v1idx).y(), mesh->getVertexNormal(v1idx).z());
		glVertex3d(mesh->getPoint(v1idx)[0], mesh->getPoint(v1idx)[1], mesh->getPoint(v1idx)[2]);
		glNormal3d(mesh->getVertexNormal(v2idx).x(), mesh->getVertexNormal(v2idx).y(), mesh->getVertexNormal(v2idx).z());
		glVertex3d(mesh->getPoint(v2idx)[0], mesh->getPoint(v2idx)[1], mesh->getPoint(v2idx)[2]);
	}
	glEnd();
}

void MeshViewerWidget::DrawHiddenLines() const
{
	glLineWidth(1.0);
	float backcolor[4];
	glGetFloatv(GL_COLOR_CLEAR_VALUE, backcolor);
	glColor4fv(backcolor);
	glDepthRange(0.01, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);
		DrawFlat();
		glEnable(GL_LIGHTING);
	}
	else
	{
		DrawFlat();
	}
	glDepthRange(0.0, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor3d(.3, .3, .3);
	DrawFlat();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void MeshViewerWidget::DrawFlatLines(void) const
{
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.5f, 2.0f);
	glShadeModel(GL_FLAT);
	//glColor3d(0.8, 0.8, 0.8);
	glColor3d(1.0, 1.0, 1.0);
	DrawFlat();
	glDisable(GL_POLYGON_OFFSET_FILL);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);
		DrawWireframe();
		glEnable(GL_LIGHTING);
	}
	else
	{
		DrawWireframe();
	}
}

void MeshViewerWidget::DrawFlat(void) const
{
	glBegin(GL_TRIANGLES);
	for (int i=0;i<mesh->NFaces();i++)
	{
		glNormal3d(mesh->getFaceNormal(i).x(), mesh->getFaceNormal(i).y(), mesh->getFaceNormal(i).z());
		int v1, v2, v3;
		mesh->getFaceVertices(i, v1, v2, v3);
		glVertex3d(mesh->getPoint(v1)[0], mesh->getPoint(v1)[1], mesh->getPoint(v1)[2]);
		glVertex3d(mesh->getPoint(v2)[0], mesh->getPoint(v2)[1], mesh->getPoint(v2)[2]);
		glVertex3d(mesh->getPoint(v3)[0], mesh->getPoint(v3)[1], mesh->getPoint(v3)[2]);
	}
	glEnd();
}

void MeshViewerWidget::DrawSmooth(void) const
{
	/*
	glColor3d(0.8, 0.8, 0.8);
	glShadeModel(GL_SMOOTH);
	glLoadName(static_cast<GLuint>(mesh.n_vertices()));
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_DOUBLE, 0, mesh.points());
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_DOUBLE, 0, mesh.vertex_normals());
	for (const auto& fh : mesh.faces())
	{
		glBegin(GL_POLYGON);
		for (const auto& fvh : mesh.fv_range(fh))
		{
			glArrayElement(fvh.idx());
		}
		glEnd();
	}
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	*/
}

void MeshViewerWidget::DrawCurvature(void) const
{
	if (curvature_v.empty() || curvature_v.size() != mesh->NVertices())
	{
		std::cerr << "ERROR: DrawColormap() values error." << std::endl;
		return;
	}
//	glEnable(GL_TEXTURE_2D);
//	colormap->bind();
	glBegin(GL_TRIANGLES);
	for (int i=0;i<mesh->NFaces();i++)
	{
//		glNormal3d(mesh->getFaceNormal(i).x(), mesh->getFaceNormal(i).y(), mesh->getFaceNormal(i).z());
		int v1, v2, v3;
		mesh->getFaceVertices(i, v1, v2, v3);
		double value = (curvature_v[v1] + curvature_v[v2] + curvature_v[v3]) / 3;
		if (0 < value && value < 0.25)
			glColor3d(0, 4 * value, 1);
		else if (0.25 < value && value < 0.5)
			glColor3d(0, 1, 1 - 4 * (value - 0.25));
		else if (0.5 < value && value < 0.75)
			glColor3d(4 * (value - 0.5), 1, 0);
		else if (value < 1)
			glColor3d(1, 1 - 4 * (value - 0.75), 0);
//		glColor3d()

//		glTexCoord2d(value,value);
		glVertex3d(mesh->getPoint(v1)[0], mesh->getPoint(v1)[1], mesh->getPoint(v1)[2]);
		glVertex3d(mesh->getPoint(v2)[0], mesh->getPoint(v2)[1], mesh->getPoint(v2)[2]);
		glVertex3d(mesh->getPoint(v3)[0], mesh->getPoint(v3)[1], mesh->getPoint(v3)[2]);
	}
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void MeshViewerWidget::DrawFaceNormal(void) const
{
	double avg_length = 0;
	for (int i = 0; i < mesh->NEdges(); i++)
		avg_length += mesh->getEdgeLength(i);
	avg_length /= mesh->NEdges();
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.5f, 2.0f);
	glShadeModel(GL_FLAT);
	glColor3d(1.0, 1.0, 1.0);
	DrawFlat();
	glDisable(GL_POLYGON_OFFSET_FILL);
	if (glIsEnabled(GL_LIGHTING))
	{
		glDisable(GL_LIGHTING);
		glColor3d(0.2, 0.2, 0.2);
		glBegin(GL_LINES);
		for (int i = 0; i < mesh->NEdges(); i++)
		{
			int v1idx, v2idx;
			mesh->getEdgeVertices(i, v1idx, v2idx);
			glNormal3d(mesh->getVertexNormal(v1idx).x(), mesh->getVertexNormal(v1idx).y(), mesh->getVertexNormal(v1idx).z());
			glVertex3d(mesh->getPoint(v1idx)[0], mesh->getPoint(v1idx)[1], mesh->getPoint(v1idx)[2]);
			glNormal3d(mesh->getVertexNormal(v2idx).x(), mesh->getVertexNormal(v2idx).y(), mesh->getVertexNormal(v2idx).z());
			glVertex3d(mesh->getPoint(v2idx)[0], mesh->getPoint(v2idx)[1], mesh->getPoint(v2idx)[2]);
		}
		glColor3d(0.0, 0.0, 1.0);
		for (int i = 0; i < mesh->NFaces(); i++)
		{
			int v1idx, v2idx, v3idx;
			mesh->getFaceVertices(i, v1idx, v2idx, v3idx);
			Eigen::Vector3d FNormal = mesh->getFaceNormal(i);
			Eigen::Vector3d Center = (mesh->getPoint(v1idx) + mesh->getPoint(v2idx) + mesh->getPoint(v3idx)) / 3;
			glVertex3d(Center.x(), Center.y(), Center.z());
			glVertex3d(Center.x() + FNormal.x()*avg_length, Center.y() + FNormal.y() * avg_length, Center.z() + FNormal.z() * avg_length);
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}
	else
	{
		glColor3d(0.2, 0.2, 0.2);
		glBegin(GL_LINES);
		for (int i = 0; i < mesh->NEdges(); i++)
		{
			int v1idx, v2idx;
			mesh->getEdgeVertices(i, v1idx, v2idx);
			glNormal3d(mesh->getVertexNormal(v1idx).x(), mesh->getVertexNormal(v1idx).y(), mesh->getVertexNormal(v1idx).z());
			glVertex3d(mesh->getPoint(v1idx)[0], mesh->getPoint(v1idx)[1], mesh->getPoint(v1idx)[2]);
			glNormal3d(mesh->getVertexNormal(v2idx).x(), mesh->getVertexNormal(v2idx).y(), mesh->getVertexNormal(v2idx).z());
			glVertex3d(mesh->getPoint(v2idx)[0], mesh->getPoint(v2idx)[1], mesh->getPoint(v2idx)[2]);
		}
		glColor3d(0.0, 0.0, 1.0);
		for (int i = 0; i < mesh->NFaces(); i++)
		{
			int v1idx, v2idx, v3idx;
			mesh->getFaceVertices(i, v1idx, v2idx, v3idx);
			Eigen::Vector3d FNormal = mesh->getFaceNormal(i);
			FNormal = FNormal / FNormal.norm() * avg_length;
			Eigen::Vector3d Center = (mesh->getPoint(v1idx) + mesh->getPoint(v2idx) + mesh->getPoint(v3idx)) / 3;
			glVertex3d(Center.x(), Center.y(), Center.z());
			glVertex3d(Center.x() + FNormal.x(), Center.y() + FNormal.y(), Center.z() + FNormal.z());
		}
		glEnd();
	}

}

void MeshViewerWidget::DrawBoundingBox(void) const
{
	float linewidth;
	glGetFloatv(GL_LINE_WIDTH, &linewidth);
	glLineWidth(2.0f);
	glColor3d(.3, .7, .3);
	glBegin(GL_LINES);
	for (const auto& i : { 0, 1 })
	{
		for (const auto& j : { 0, 1 })
		{
			for (const auto& k : { 0, 1 })
			{
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(~i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], ~j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], k ? ptMin[2] : ptMax[2]);
				glVertex3d(i ? ptMin[0] : ptMax[0], j ? ptMin[1] : ptMax[1], ~k ? ptMin[2] : ptMax[2]);
			}
		}
	}
	glEnd();
	glLineWidth(linewidth);
}

void MeshViewerWidget::DrawBoundary(void) const
{
	float linewidth;
	glGetFloatv(GL_LINE_WIDTH, &linewidth);
	glLineWidth(5.0f);
	glColor3d(0, 0, 1);
	glBegin(GL_LINES);
	for (int i=0;i<mesh->NEdges();i++)
	{
		if (mesh->isBoundary(i))
		{
			int v1idx, v2idx;
			mesh->getEdgeVertices(i, v1idx, v2idx);
			glNormal3d(mesh->getVertexNormal(v1idx).x(), mesh->getVertexNormal(v1idx).y(), mesh->getVertexNormal(v1idx).z());
			glVertex3d(mesh->getPoint(v1idx)[0], mesh->getPoint(v1idx)[1], mesh->getPoint(v1idx)[2]);
			glNormal3d(mesh->getVertexNormal(v2idx).x(), mesh->getVertexNormal(v2idx).y(), mesh->getVertexNormal(v2idx).z());
			glVertex3d(mesh->getPoint(v2idx)[0], mesh->getPoint(v2idx)[1], mesh->getPoint(v2idx)[2]);
		}
	}
	glEnd();
	glLineWidth(linewidth);
}

void MeshViewerWidget::MapCurvature(const std::vector<double>& values)
{
	curvature_v.clear();
	curvature_v.resize(values.size());
	//¼òµ¥ÅÅÐò
	struct k_v
	{
		k_v(int i, double v) { vid = i, value = v; };
		bool operator> (k_v& b)
		{
			return value > b.value;
		}
		bool operator< (k_v& b)
		{
			return value < b.value;
		}
		int vid;
		double value;
	};
	std::vector<k_v> curvature;
	for (int i = 0; i < values.size(); i++)
		curvature.push_back(k_v(i, values[i]));
	std::sort(curvature.begin(), curvature.end());
	for (int i = 0; i < values.size(); i++)
	{
		curvature_v[curvature[i].vid] = double(i) / values.size();
	}
	/*
	double maxV, minV;
	double sum=std::accumulate(values.begin(), values.end(),0.0);
	double mean = sum / values.size();
	double accum = 0.0;
	std::for_each(std::begin(values), std::end(values), [&](const double d)
		{
			accum += (d - mean) * (d - mean);
		});
	//double stdev = sqrt(accum / values.size() - 1);
	double stdev = 0.01;
	maxV = mean + stdev; minV = mean - stdev;
	for (int i = 0; i < values.size(); i++)
	{
		curvature_v[i] = (values[i] - minV) / (maxV - minV);
	}
	std::cout << 123 << std::endl;*/
}