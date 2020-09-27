#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

class MyMesh
{
public:
    struct Point
    {
        //        Eigen::Vector3d p;
        double v[3];
        Point() { v[0] = v[1] = v[2] = 0; };
        Point(double x)
        {
            v[0] = v[1] = v[2] = x;
        }
        Point(double x, double y, double z)
        {
            v[0] = x; v[1] = y; v[2] = z;
        }
        void Minimize(Point& p)
        {
            if (v[0] > p.v[0]) v[0] = p.v[0];
            if (v[1] > p.v[1]) v[1] = p.v[1];
            if (v[2] > p.v[2]) v[2] = p.v[2];
        }
        void Maximize(Point& p)
        {
            if (v[0] < p.v[0]) v[0] = p.v[0];
            if (v[1] < p.v[1]) v[1] = p.v[1];
            if (v[2] < p.v[2]) v[2] = p.v[2];
        }
        double operator[](int i) const
        {
            if (0 <= i && i <= 2)
                return v[i];
            else
                std::cerr << "Point的下标值溢出" << std::endl;
        }
        Point operator+(Point& p)
        {
            return Point(v[0] + p.v[0], v[1] + p.v[1], v[2] + p.v[2]);
        }
        void operator=(double value)
        {
            v[0] = v[1] = v[2] = value;
        }
        void operator=(Point& p)
        {
            v[0] = p.v[0]; v[1] = p.v[1]; v[2] = p.v[2];
        }
        void operator=(Eigen::Vector3d& p)
        {
            v[0] = p.x(); v[1] = p.y(); v[2] = p.z();
        }
        Point operator-(Point& p)
        {
            return Point(v[0] - p.v[0], v[1] - p.v[1], v[2] - p.v[2]);
        }
        Point operator*(double t)
        {
            return Point(v[0] * t, v[1] * t, v[2] * t);
        }
        Point operator/(double t)
        {
            return Point(v[0] / t, v[1] / t, v[2] / t);
        }
        double norm()
        {
            return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        }
        Eigen::Vector3d toEigen3d()
        {
            return Eigen::Vector3d(v[0], v[1], v[2]);
        }
    };

private:

public:
    virtual bool Load(std::string) = 0;//读入网格
    virtual bool Write(std::string) = 0;//导出网格
    virtual void Clear() = 0;//清理网格
    virtual void UpdateNormals() = 0;
    virtual bool VerticesEmpty() = 0;
    virtual int NVertices() const = 0;
    virtual int NEdges() const = 0;
    virtual int NFaces() const = 0;
    virtual double CalcEdgeLength(int i) = 0;
    virtual double CalcFaceArea() = 0;
    virtual Point getPoint(int i) const = 0;
    virtual Eigen::Vector3d getVertexNormal(int i) const = 0;
    virtual Eigen::Vector3d getFaceNormal(int i) const = 0;
    virtual void getEdgeVertices(int e, int& v1, int& v2) const = 0;
    virtual double getEdgeLength(int e) const = 0;
    virtual void getFaceVertices(int f, int& v1, int& v2, int& v3) const = 0;
    virtual void getFaceAngles(int f, double& angle1, double& angle2, double& angle3) const = 0;
    virtual bool isBoundary(int) const = 0;
    virtual void SetVerticeNewCoord(int, Eigen::Vector3d) =0;
    virtual Eigen::Vector3d getVertexCoord(int)  = 0;
    
    virtual void ComputeLaplacian(int) = 0;//构建Laplacian矩阵

private:
    double ComputeArea(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3);
    Eigen::Vector3d ComputeTriangleCenter(Eigen::Vector3d normal, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);

public:
  //  void ComputeLaplacian(int kind);
    void ComputeLocalAveragingRegion(int kind);//获取点的平均区域，暂时是获取面积
    void UpdateMeanCurvature();
    void UpdateGaussianCurvature();
    void MakeNoise();
    std::vector<double> VCurvature;
    void getVCurvature(std::vector<double>& c) { c = VCurvature; };
    std::vector<double> V_LocalAverageRegionArea;

protected:
        Eigen::Matrix3d Vertices;//网格顶点
        Eigen::Matrix3d FaceNormal;//面法向，唯一
        Eigen::Matrix3d VertexNormal;//顶点法向，多种加权方式
        Eigen::MatrixXd Laplacian;//拉普拉斯矩阵，两种uniform或者cotangent

};