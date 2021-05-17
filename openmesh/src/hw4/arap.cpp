 #include <Eigen/Dense>
#include <Eigen/Sparse>

void CalcCotLaplacian(T& mesh, Eigen::SparseMatrix<double>& lap,std::vector<double>& cots)
{
	cots.resize(mesh.n_halfedges());
	std::vector<Eigen::Triplet<double>> trivec;
	for (T::HalfedgeHandle heh : mesh.halfedges())
	{
		if (mesh.is_boundary(heh))
			continue;
		T::VertexHandle v1 =mesh.from_vertex_handle(heh);
		T::VertexHandle v2 =mesh.to_vertex_handle(heh);
		T::VertexHandle v0 =mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));

		OpenMesh::Vec3d e01 = mesh.point(v1) - mesh.point(v0);
		OpenMesh::Vec3d e02 = mesh.point(v2) - mesh.point(v0);
		double cotangle = (e02 | e01) / (e01 % e02).norm();

		cots[heh.idx()] = cotangle;
		trivec.emplace_back(v1.idx(), v1.idx(), cotangle);
		trivec.emplace_back(v2.idx(), v2.idx(), cotangle);
		trivec.emplace_back(v1.idx(), v2.idx(), -cotangle);
		trivec.emplace_back(v2.idx(), v1.idx(), -cotangle);
	}
	lap.resize(mesh.n_vertices(), mesh.n_vertices());
	lap.setFromTriplets(trivec.begin(), trivec.end());
}

//��������ľֲ����꣬�ֲ�������һ��(nv,6)�ľ���
void CalcLocalCoord(T& mesh, Eigen::MatrixXd& coord)
{
	coord.resize(mesh.n_faces(), 6);
	for (T::FaceHandle fh : mesh.faces())
	{
		T::Normal n = mesh.normal(fh);
		T::FVIter fvi = mesh.fv_begin(fh);
		T::Point p0 = mesh.point(fvi);
		fvi++;
		T::Point p1 = mesh.point(fvi);
		fvi++;
		T::Point p2 = mesh.point(fvi);

		OpenMesh::Vec3d e01 = p1 - p0;
		OpenMesh::Vec3d e02 = p2 - p0;

		OpenMesh::Vec3d x_ = e01.normalized();
		OpenMesh::Vec3d y_ = (n % x_);

		coord.row(fh.idx()) << 0, 0, e01.norm(), 0, e02 | x_, e02 | y_;
	}
}

void CalcTutte(T& mesh,Eigen::MatrixX2d& result)
{
	int F_N = mesh.n_faces();
	int V_N = mesh.n_vertices();

	result.resize(V_N, 2);

	double area_sum = 0;
	for (int i = 0; i < F_N; ++i)
	{
		T::FaceHandle fh = mesh.face_handle(i);
		T::FVIter fvi = mesh.fv_begin(fh);
		T::Point p0 = mesh.point(fvi);
		fvi++;
		T::Point p1 = mesh.point(fvi);
		fvi++;
		T::Point p2 = mesh.point(fvi);

		OpenMesh::Vec3d e01 = p1 - p0;
		OpenMesh::Vec3d e02 = p2 - p0;
		area_sum+= 0.5 * (e01 % e02).norm();
	}

	int n_border_v = 0;
	int st = -1;//��һ���߽��
	T::HalfedgeHandle he_start;
	for (T::HalfedgeHandle heh : mesh.halfedges())
		if (mesh.is_boundary(heh))
		{
			he_start = heh;
			break;
		}
	T::HalfedgeHandle he_it = he_start;
	do {
		he_it = mesh.next_halfedge_handle(he_it);
		n_border_v++;
	} while (he_it != he_start);

	double delta_angle = 2 * M_PI / n_border_v;
	double area_1_factor = sqrt(area_sum / M_PI);

	Eigen::VectorXd position_of_mesh;
	position_of_mesh.resize(2 * V_N);
	for (int i = 0; i < n_border_v; ++i)
	{
		T::VertexHandle v_h = mesh.to_vertex_handle(he_start);
		position_of_mesh(v_h.idx()) = area_1_factor * cos(i * delta_angle);
		position_of_mesh(v_h.idx() + V_N) = area_1_factor * sin(-i * delta_angle);
		he_start = mesh.next_halfedge_handle(he_start);
	}
	std::vector<Eigen::Triplet<double>> triple_list;
	Eigen::VectorXd bu = Eigen::VectorXd::Zero(V_N);
	Eigen::VectorXd bv = Eigen::VectorXd::Zero(V_N);
	for (T::VertexHandle vh : mesh.vertices())
	{
		int vid = vh.idx();
		if (mesh.is_boundary(vh))
		{
			triple_list.push_back(Eigen::Triplet<double>(vid, vid, 1));
			T::Point p = mesh.point(vh);
			bu(vid) = position_of_mesh[vid];
			bv(vid) = position_of_mesh[vid + V_N];
		}
		else
		{//����1-����
			for (T::VertexHandle vvh : mesh.vv_range(vh))
			{
				int vvid = vvh.idx();
				triple_list.push_back(Eigen::Triplet<double>(vid,vvid, -1));
			}
			triple_list.push_back(Eigen::Triplet<double>(vid, vid, mesh.valence(vh)));
		}
	}

	Eigen::SparseMatrix<double> LL(V_N, V_N);
	LL.setFromTriplets(triple_list.begin(), triple_list.end());
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;Ҫ���������
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(LL);//ִ��Ԥ�ֽ�
	Eigen::VectorXd xu = solver.solve(bu);
	Eigen::VectorXd xv = solver.solve(bv);

	result.col(0) = xu;
	result.col(1) = xv;
}

int main(int argc, char* argv[])
{
	std::cout << "===============\tARAP\t===============\n\n";
@ -13,7 +149,6 @@ int main(int argc, char* argv[])
	std::string mpath;
	T mesh;
	std::stringstream ss;
	std::cout << argv[1];
	if (argc != 2)
	{
		std::cout << "������Ŀ������ȷ\n";
@ -24,9 +159,122 @@ int main(int argc, char* argv[])
		ss << argv[1];
		ss >> mpath;
	}
	OpenMesh::IO::read_mesh(mesh, ((argc > 1) ? argv[1] : mpath + "alien_open.off"));
	OpenMesh::IO::read_mesh(mesh, ((argc > 1) ? mpath : mpath + "alien_open.off"));
	int nv = mesh.n_vertices();
	std::cout << nv;
	mesh.request_face_normals();
	mesh.update_face_normals();
	Eigen::MatrixXd LocalCoord;//���¾ֲ�����
	CalcLocalCoord(mesh, LocalCoord);

	Eigen::MatrixX2d UV;//����tutte
	CalcTutte(mesh, UV);

	Eigen::SparseMatrix<double> Laplacian;//����cotanȨ��Laplacian
	std::vector<double> cots;
	CalcCotLaplacian(mesh, Laplacian,cots);
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(Laplacian);

	std::vector<Eigen::Matrix2d> Lts;
	Lts.resize(mesh.n_faces());

	for (int iter = 0; iter < 100; iter++)
	{
		//local�׶Σ�������ת����Lts
		for (T::FaceHandle fh : mesh.faces())
		{
			T::FVIter fvi = mesh.fv_begin(fh);
			int fid = fh.idx();
			int vid0 = fvi->idx();
			fvi++;
			int vid1 = fvi->idx();
			fvi++;
			int vid2 = fvi->idx();

			Eigen::Matrix2d P, S, J;
			//��Jacobian����
			P << UV(vid1, 0) - UV(vid0, 0), UV(vid2, 0) - UV(vid0, 0), UV(vid1, 1) - UV(vid0, 1), UV(vid2, 1) - UV(vid0, 1);
			S << LocalCoord(fid, 2) - LocalCoord(fid, 0), LocalCoord(fid, 4) - LocalCoord(fid, 0),
				LocalCoord(fid, 3) - LocalCoord(fid, 1), LocalCoord(fid, 5) - LocalCoord(fid, 1);
			J = P * S.inverse();

			//Jacobian������SVD�ֽ�
			Eigen::JacobiSVD<Eigen::Matrix2d> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

			Eigen::Matrix2d U = svd.matrixU();//������ֵ����
			Eigen::Matrix2d V = svd.matrixV();//������ֵ����

			Eigen::Matrix2d R = U * V.transpose();

			if (R.determinant() < 0)//�����ת��������ֵ����ĵڶ���ȡ����
			{
				U(0, 1) = -U(0, 1);
				U(1, 1) = -U(1, 1);
				R = U * V.transpose();
			}
			Lts[fid] = R;//��ת����
		}

		//Global�����¶���λ�ã�����һ��2�εĺ�����ֱ���󵼲�������Ϊ0���ɡ�
		Eigen::VectorXd bu, bv;
		bu.setZero(nv);
		bv.setZero(nv);

		for (T::FaceHandle fh:mesh.faces())
		{
			int fid = fh.idx();
			T::Normal n = mesh.normal(fh);
			T::FVIter fvi = mesh.fv_begin(fh);
			int vid0 = fvi->idx();
			fvi++;
			int vid1 = fvi->idx();
			fvi++;
			int vid2 = fvi->idx();

			T::HalfedgeHandle he2 = mesh.halfedge_handle(fh);
			T::HalfedgeHandle he0 = mesh.next_halfedge_handle(he2);
			T::HalfedgeHandle he1 = mesh.next_halfedge_handle(he0);

			Eigen::Vector2d e0, e1, e2;
			e0 << LocalCoord(fid, 2), LocalCoord(fid, 3);
			e1 << LocalCoord(fid, 4) - LocalCoord(fid, 2), LocalCoord(fid, 5) - LocalCoord(fid, 3);
			e2 << -LocalCoord(fid, 4), -LocalCoord(fid, 5);
			Eigen::Vector2d b0 = cots[he0.idx()] * Lts[fid] * e0;
			bu[vid0] -= b0[0];
			bv[vid0] -= b0[1];

			bu[vid1] += b0[0];
			bv[vid1] += b0[1];
			Eigen::Vector2d b1 = cots[he1.idx()] * Lts[fid] * e1;
			bu[vid1] -= b1[0];
			bv[vid1] -= b1[1];

			bu[vid2] += b1[0];
			bv[vid2] += b1[1];

			Eigen::Vector2d b2 = cots[he2.idx()] * Lts[fid] * e2;
			bu[vid2] -= b2[0];
			bv[vid2] -= b2[1];

			bu[vid0] += b2[0];
			bv[vid0] += b2[1];
		}
		//global solve
		UV.col(0) = solver.solve(bu);
		UV.col(1) = solver.solve(bv);
	}


	for (int i = 0; i < mesh.n_vertices(); i++)
	{
		T::VertexHandle vh = mesh.vertex_handle(i);
		T::Point p(UV(i, 0), UV(i, 1), 0);
		mesh.set_point(vh, p);
	}

	OpenMesh::IO::write_mesh(mesh, "output.obj");


