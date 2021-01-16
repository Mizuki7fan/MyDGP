#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <iostream>
#include <fstream>
typedef CGAL::Simple_cartesian<double>			Kernel;
typedef Kernel::Point_3										Point;
typedef CGAL::Surface_mesh<Point>					Mesh;
typedef Mesh::Vertex_iterator								Vertex_iterator;

int main(int argc,char* argv[])
{
	std::cout << "===============Tutte==============\n";
	Mesh sm1;
	std::ifstream in1((argc > 1) ? argv[1] : "D:/MyRepo/DigitalGeometryPrecessing/MyDGP/mesh/alien_open.off");
	in1 >> sm1;
	std::cout << sm1.num_vertices() << std::endl;
	std::vector<int> Bnd_v;
	for (Mesh::vertex_index vd : sm1.vertices())
	{
		Mesh::vertex_index vi(1);
		if (sm1.is_border(vd))//寻找所有边界点
			
			std::cout << vd.idx() << std::endl;
	}
	if (Bnd_v.size() == 0)
	{
		std::cerr << "网格是封闭的\n";
		return EXIT_FAILURE;
	}
	std::vector<int> Bnd_v_inorder;//令边界点有序


	return EXIT_SUCCESS;



	
}