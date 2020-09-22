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
//            return Point(v[0] + p.v[0], v[1] + p.v[1], v[2] + p.v[2]);
        }
        Point operator-(Point& p)
        {
            return Point(v[0] - p.v[0], v[1] - p.v[1], v[2] - p.v[2]);
        }
        Point operator*(double t)
        {
            return Point(v[0] * t, v[1] * t, v[2] * t);
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

    struct Vertice
    {
        int id;
        Point p;

    };
private:

public:
    virtual bool Load(std::string) = 0;
    virtual bool Write(std::string) = 0;
    virtual void Clear() = 0;
    virtual void UpdateNormals() = 0;
    virtual bool VerticesEmpty() = 0;
    virtual int NVertices() const = 0;
    virtual int NEdges() const = 0;
    virtual int NFaces() const = 0;
    virtual double CalcEdgeLength(int i) = 0;
    virtual Point getPoint(int i) const = 0;
    virtual Eigen::Vector3d getVertexNormal(int i) const = 0;
    virtual Eigen::Vector3d getFaceNormal(int i) const = 0;
    virtual void getEdgeVertices(int e, int& v1, int& v2) const = 0;
    virtual void getFaceVertices(int f, int& v1, int& v2, int& v3) const = 0;
    virtual bool isBoundary(int) const = 0;
};