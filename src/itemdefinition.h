#pragma once

enum LAR_KIND
{
	ND,BARYCENTRIC, VORONOI, MIXED
};

enum LAPLACIAN_KIND
{
	ND,UNIFORM, COTANGENT
};

enum CURVATURE_KIND
{
	ND,MEAN,ABSOLUTEMEAN,GAUSSIAN
};

class property
{
public:
	LAR_KIND lar_kind;
	LAPLACIAN_KIND lap_kind;
	CURVATURE_KIND cvture_kind;
};