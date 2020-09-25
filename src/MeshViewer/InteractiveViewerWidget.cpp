#include <QtCore>
#include <QtGui>
#include "InteractiveViewerWidget.h"
#include "ANN/ANN.h"

InteractiveViewerWidget::InteractiveViewerWidget(QWidget* parent /* = 0 */)
	:MeshViewerWidget(parent)
{
	clearSelectedData();
	kdTree = NULL;
}

InteractiveViewerWidget::~InteractiveViewerWidget()
{
	if (kdTree) delete kdTree;
}


void InteractiveViewerWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->mimeData()->hasFormat("text/uri-list"))
	{
		event->acceptProposedAction();
	}
}

void InteractiveViewerWidget::dropEvent(QDropEvent* event)
{
	QList<QUrl> urls = event->mimeData()->urls();
	if (urls.isEmpty())
		return;
	QString fileName = urls.first().toLocalFile();
	if (fileName.isEmpty())
		return;

	if (fileName.endsWith(".off") || fileName.endsWith(".obj") || fileName.endsWith(".stl") || fileName.endsWith(".ply"))
	{
		if (LoadMesh(fileName.toStdString()))
		{
			emit(LoadMeshOKSignal(true, fileName));
		}
		else
		{
			emit(LoadMeshOKSignal(false, "No Mesh"));
		}
	}
}
