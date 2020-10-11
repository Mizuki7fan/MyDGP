#pragma once
#include "MeshDefinition/MyMesh.h"

class DGPAlgorithm
{
public:
	//算网格的点的曲率，参数为局部平均区域的类型、曲率的类型
	static void ComputeCurvature(int LAR_Kind, int CurvatureKind, MyMesh& mesh, std::vector<double>& curvature);
	
	//制造网格噪音
	static void MakeNoise(MyMesh& mesh);
	//进行Fairing操作
	static void DoFairing(MyMesh& mesh, int power,int lap_kind);
	//进行Smoothing操作
	static void DoSmoothing(MyMesh& mesh, int laplacekind,int integrationkind);
	static void DoBilateralDenoising(MyMesh& mesh, double stdevs,double stdevr);
	static void DoBilateralNormalFiltering(MyMesh& mesh, double stdevs, double stdevr);
	static void CalcTutte(MyMesh& mesh);
	static void CalcLSCM(MyMesh& mesh);
private:

};