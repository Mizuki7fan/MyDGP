#pragma once
#include "MeshViewerWidget.h"
#include "ANN/ANN.h"
class InteractiveViewerWidget : public MeshViewerWidget
{
	Q_OBJECT
public:
	InteractiveViewerWidget(QWidget* parent = 0);
	~InteractiveViewerWidget();
	void clearSelectedData()
	{
		selectedVertex.clear();
		selectedFace.clear();
		selectedEdge.clear();
	};

public:
	enum { TRANS, POINTPICK, VERTEXPICK, EDGEPICK, FACEPICK, EDGECOLLAPSE, EDGEFLIP, EDGESPLIT, MOVE, T2_MODE, N_MODE };
	void setMouseMode(int mm);
	int mouseMode() const { return mouse_mode_; }

protected:
	void pick_vertex(int x, int y);
	void pick_face(int x, int y);
	void pick_edge(int x, int y);
	void pick_point(int x, int y);

protected:
	//virtual void mousePressEvent(QMouseEvent* _event);
	//virtual void mouseReleaseEvent(QMouseEvent* _event);
	//virtual void mouseMoveEvent(QMouseEvent* _event);
	//virtual void wheelEvent(QWheelEvent* _event);
	int mouse_mode_;
	int t2_mode_;
	void dragEnterEvent(QDragEnterEvent *event);
	void dropEvent(QDropEvent *event);
	ANNkd_tree* kdTree;
	double selectedPoint[3];
	std::vector<int> selectedVertex;
	int lastestVertex;
	std::vector<int> selectedFace;
	int lastestFace;
	std::vector<int> selectedEdge;
	int lastestEdge;
};