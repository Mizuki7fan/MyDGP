#pragma once
#include <iostream>
#include <fstream>
#include "../general/MeshDefinition.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

void find_neighbour(T& m, int c,int& prev,int&next)
{
	//输入一个点，寻找其前后的边界点
	int nei[2];
	int i = 0;
	T::VertexHandle vh = m.vertex_handle(c);
	//这里要用ve_range而不能用vv_range，因为可能会出现某个三角形的三个点都是边界点的情况
	for (auto eh : m.ve_range(vh))
	{
		if (m.is_boundary(eh))
		{
			int v1 = m.to_vertex_handle(m.halfedge_handle(eh,0)).idx();
			int v2 = m.to_vertex_handle(m.halfedge_handle(eh, 1)).idx();
			nei[i] = (v1 == c) ? v2 : v1;
			i++;
		}
	}
	prev = nei[0];
	next = nei[1];
}


int main(int argc,char* argv[])
{
	std::cout << "===============\tTutte\t===============\n\n";
	std::cout << "example:\ttutte.exe\talien_open.off\n\n";
	std::cout << "=======================================\n";
	std::string mpath = "D:/MyRepo/DigitalGeometryPrecessing/MyDGP/mesh/";
	T mesh;
	OpenMesh::IO::read_mesh(mesh, ((argc > 1) ? argv[1] : mpath+"alien_open.off"));

	//先把网格边界点按顺序投影到一个圆上，获得一组**有序的**边界点
	//为了判定是否有多组边界，需要先统计边的总数量
	int st = -1;//第一个边界点
	int n_border_v = 0;
	for (auto vh : mesh.vertices())
	{
		if (mesh.is_boundary(vh))
		{
			n_border_v++;
			st = vh.idx();
		}
	}
	std::vector<int> Bnd_v_inorder;
	Bnd_v_inorder.push_back(st);
	int prev_v = -1;
	int next_v = -1;//下一个点的id
	find_neighbour(mesh, st, prev_v, next_v);
	int center_v = st;
	while (next_v!=st)
	{
		Bnd_v_inorder.push_back(next_v);
		prev_v = center_v;
		center_v = next_v;
		int v1, v2;
		find_neighbour(mesh, center_v, v1, v2);
		next_v = (v1 == prev_v) ? v2 : v1;
	}
	if (Bnd_v_inorder.size() != n_border_v)
	{
		std::cerr << "网格并非圆盘拓扑" << std::endl;
		return 0;
	}
	//构建矩阵
	double r = 5.0;
	int bnd_size = n_border_v;
	Eigen::MatrixXd A, b;
	A.resize(mesh.n_vertices(), mesh.n_vertices());
	A.setZero();
	b.resize(mesh.n_vertices(), 3);
	b.setZero();
	for (double idx = 0; idx < bnd_size; idx++)
	{
		int vid = Bnd_v_inorder[idx];
		A(vid, vid) = 1;
		double theta = 2 * M_PI / bnd_size * idx;
		b(vid, 0) = r * cos(theta);
		b(vid, 1) = r * sin(theta);
	}
	for (T::VertexHandle vh : mesh.vertices())
	{
		if (mesh.is_boundary(vh))
			continue;
		//否则求vh的邻域
		int nei_count = 0;
		for (T::VertexHandle vnei : mesh.vv_range(vh))
			nei_count++;
		for (T::VertexHandle vnei : mesh.vv_range(vh))
		{
			A(vh.idx(), vnei.idx()) = 1.0;
		}
		A(vh.idx(), vh.idx()) = -nei_count;//这么写可以获得一个对称矩阵
	}
	Eigen::SparseMatrix<double> LL = A.sparseView();
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;要求矩阵正定
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(LL);
	Eigen::MatrixXd res = solver.solve(b);//参数化结果的点
	//Eigen::MatrixXd res = A.inverse() * b;
	for (T::VertexHandle vh : mesh.vertices())
	{
		int idx = vh.idx();
		T::Point p(res(idx, 0), res(idx, 1), res(idx, 2));
		mesh.set_point(vh, p);
	}
	
	OpenMesh::IO::write_mesh(mesh, "output.obj");
	return 0;
}