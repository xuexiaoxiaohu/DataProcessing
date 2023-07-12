#pragma once
// QT
#include <QVector>
#include <QVector3D>
// PCL
#include <pcl/io/ply_io.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/vtk_lib_io.h>
//VTK
#include <vtkTriangleFilter.h>
#include <vtkFillHolesFilter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkImageStencil.h>
#include <vtkMarchingCubes.h>
#include <vtkImageGaussianSmooth.h>

#define SCR_WIDTH			800
#define SCR_HEIGHT			700

#define MESH_GRTH_SIZE		4000
#define MIN_PTS_SIZE_REQD	400

#define	ITERATS				50
#define	NUM_NEIGHS			50
#define MAX_HOLE_SIZE		10000

#define  VOLUME_MAX			1000
#define  DELTA				0.5

#define SCROLL_SEN			0.01
#define BRUSH_PARAM_MAX		100
#define BRUSH_PARAM_MIN		-100
#define PARAM_A				0.45
#define PARAM_B				55
class DataProcessing {
public:
	DataProcessing() {};
	~DataProcessing() {};

	void loadPointData(QString path);
	void addNormal(pcl::PolygonMesh &inMesh);
	void getMaxMinPoint(std::vector<QVector3D> data);
	void poly2tri(std::string src, std::string dst);

	int getNearestVertexIndex(QVector3D worldPos, std::vector<QVector3D> glVtx);
	std::vector<int> getIndicesFromRadiusSearch(pcl::PolygonMesh mesh, pcl::PointXYZ searchPoint, float radius = 4);
	void eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete);
	void fillMesh(pcl::PolygonMesh& mesh);
	void getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh, float radius);
	//Remeshing: Isotropic Explicit Remeshing
	void isoExpRemeshing(const char* srcPath,const char* dstPath);
	QVector<QVector3D>	pointData;
	std::vector<float> glMeshData;
	QVector3D maxPoint,minPoint;

	//lr 
	void getRenderData(pcl::PolygonMesh& mesh);
	void getClipPlaneMesh(pcl::PolygonMesh& mesh, double a, double b, double c, QVector3D p);
	void getClipPlane_X0Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_X1Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Y0Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Y1Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Z0Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Z1Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void clipPlaneMesh(pcl::PolygonMesh& mesh, double a, double b, double c, QVector3D p);
	QVector<QVector3D>	worldPos;
	void polyClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos);
	void boxClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos, double rayStart[3]);

};