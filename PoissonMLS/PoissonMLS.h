#pragma once

#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include <maya/MFnPlugin.h>

#include <numeric>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>

#include "affinelib.h"
#include "tetrise.h"
#include "MeshMaya.h"
#include "laplacian.h"
#include "blendAff.h"
#include "distance.h"
#include "deformerConst.h"

using namespace Eigen;

class MLSDeformerNode : public MPxDeformerNode{
public:
    MLSDeformerNode(): numPrb(0)  {};
    virtual MStatus deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex );
    static  void*   creator();
    static  MStatus initialize();
 
    static MTypeId      id;
    static MString      nodeName;
    static MObject      aMLSMode;
    static MObject      aNormExponent;
    static MObject      aWeightMode;
    static MObject		aEffectRadius;
    static MObject      aNormaliseWeight;
    static MObject      aAreaWeighted;
    static MObject      aNeighbourWeighting;
    static MObject      aPositiveWeight;
    static MObject      aCtlPoints;
    static MObject      aInitCtlPoints;
    static MObject      aRecompMLS;
    static MObject      aRecompARAP;
    static MObject      aPoisson;
    static MObject      aCtlWeight;
    static MObject      aTetMode;
    static MObject      aIteration;
    static MObject      aConstraintMode;
    static MObject      aConstraintWeight;
    static MObject      aConstraintRadius;
    
    
    
private:
    Distance    D;
    Laplacian mesh;
    std::vector< std::vector<double> > w;   // weight for the target mesh points
    int numPrb;
    int isError;
    // mesh
    std::vector< edge > edgeList;
    std::vector<vertex> vertexList;
    std::vector<int> faceList;
    std::vector<Vector3d> pts;
    std::vector<T> constraint;
    std::vector<Vector3d> icenter;
    std::vector<Vector3d> tetCenter;  // location of tet
    //
    std::vector<MatrixXd> p;
    std::vector<Matrix3d> PPI;
    std::vector<double> trPP;
    std::vector<Matrix4d> Q;
};

