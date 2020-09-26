#include "surfacemeshprocessing.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	QFont f("Microsoft YaHei", 10, 50);
	app.setFont(f);
	QSurfaceFormat format;
	format.setSamples(16);
	QSurfaceFormat::setDefaultFormat(format);
	SurfaceMeshProcessing mainWin;
	mainWin.show();
	return app.exec();
}
