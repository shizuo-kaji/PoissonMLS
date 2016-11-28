/**
 * @file PoissonMLS.cpp
 * @brief Poisson Moving Least Square Deformer plugin for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section (included) AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.10
 * @date  14/Nov/2016
 * @author Shizuo KAJI
 */

#include "StdAfx.h"
#include "PoissonMLS.h"

using namespace Eigen;
using namespace AffineLib;
using namespace Tetrise;

MTypeId MLSDeformerNode::id( 0x00000333 );
MString MLSDeformerNode::nodeName("PoissonMLS");
MObject MLSDeformerNode::aMLSMode;
MObject MLSDeformerNode::aNormExponent;
MObject MLSDeformerNode::aWeightMode;
MObject MLSDeformerNode::aEffectRadius;
MObject MLSDeformerNode::aNormaliseWeight;
MObject MLSDeformerNode::aAreaWeighted;
MObject MLSDeformerNode::aNeighbourWeighting;
MObject MLSDeformerNode::aPositiveWeight;
MObject MLSDeformerNode::aCtlPoints;
MObject MLSDeformerNode::aInitCtlPoints;
MObject MLSDeformerNode::aRecompMLS;
MObject MLSDeformerNode::aRecompARAP;
MObject MLSDeformerNode::aPoisson;
MObject MLSDeformerNode::aCtlWeight;
MObject MLSDeformerNode::aTetMode;
MObject MLSDeformerNode::aIteration;
MObject MLSDeformerNode::aConstraintMode;
MObject MLSDeformerNode::aConstraintWeight;
MObject MLSDeformerNode::aConstraintRadius;



void* MLSDeformerNode::creator() { return new MLSDeformerNode; }

MStatus MLSDeformerNode::deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex )
{
    MStatus status;
    MThreadUtils::syncNumOpenMPThreads();    // for OpenMP
    short weightMode = data.inputValue( aWeightMode ).asShort();
    bool areaWeighted = data.inputValue( aAreaWeighted ).asBool();
    double effectRadius = data.inputValue( aEffectRadius ).asDouble();
    double normExponent = data.inputValue( aNormExponent ).asDouble();
    short MLSMode = data.inputValue(aMLSMode).asShort();
    // purge disconnected connections
    MArrayDataHandle hCtlPoints = data.inputArrayValue(aCtlPoints);
    MArrayDataHandle hInitCtlPoints = data.inputArrayValue(aInitCtlPoints);
    if(hCtlPoints.elementCount() > hInitCtlPoints.elementCount() || hCtlPoints.elementCount() == 0 || MLSMode == MLS_OFF){
        return MS::kSuccess;
    }else if(hCtlPoints.elementCount() < hInitCtlPoints.elementCount()){
        std::set<int> indices;
        for(int i=0;i<hInitCtlPoints.elementCount();i++){
            hInitCtlPoints.jumpToArrayElement(i);
            indices.insert(hInitCtlPoints.elementIndex());
        }
        for(int i=0;i<hCtlPoints.elementCount();i++){
            hCtlPoints.jumpToArrayElement(i);
            indices.erase(hCtlPoints.elementIndex());
        }
        deleteAttr(data, aInitCtlPoints, indices);
        deleteAttr(data, aCtlWeight, indices);
    }
    bool isNumProbeChanged = (numPrb != hCtlPoints.elementCount());
    numPrb = hCtlPoints.elementCount();
    // read control points from probes
    std::vector<Vector3d> initCtlPoints(numPrb), ctlPoints(numPrb);
    readVectorArray(hInitCtlPoints, initCtlPoints);
    readVectorArray(hCtlPoints, ctlPoints);
    
    // load target mesh pts
    MPointArray Mpts;
    itGeo.allPositions(Mpts);
    int numPts = Mpts.length();
    std::vector<Vector3d> pts(numPts);
    for(int j=0; j<numPts; j++ ){
        Mpts[j] *= localToWorldMatrix;
        pts[j] << Mpts[j].x, Mpts[j].y, Mpts[j].z;
    }
    
    // Poisson ?
    bool poisson = data.inputValue( aPoisson ).asBool();
    short tetMode = data.inputValue(aTetMode).asShort();
    short constraintMode = data.inputValue(aConstraintMode).asShort();
    short numIter = data.inputValue( aIteration ).asShort();
    int num;
    bool recompARAP = !data.isClean(aRecompARAP);
    if(poisson){
        if(recompARAP || mesh.numTet == 0){
            // make tetrahedral structure
            getMeshData(data, input, inputGeom, mIndex, tetMode, pts, mesh.tetList, faceList, edgeList, vertexList, mesh.tetMatrix, mesh.tetWeight);
            mesh.dim = removeDegenerate(tetMode, numPts, mesh.tetList, faceList, edgeList, vertexList, mesh.tetMatrix);
            makeTetMatrix(tetMode, pts, mesh.tetList, faceList, edgeList, vertexList, mesh.tetMatrix, mesh.tetWeight);
            makeTetCenterList(tetMode, pts, mesh.tetList, tetCenter);
            mesh.numTet = (int)mesh.tetList.size()/4;
            mesh.computeTetMatrixInverse();
        }
        num = mesh.numTet;
    }else{
        num = numPts;
    }
    
    
    // MLS without Poisson
    if(!data.isClean(aRecompMLS) || isNumProbeChanged){
        
        // load weghts of ctl points
        std::vector<double> ctlWeight;
        ctlWeight.assign(numPrb,1.0);
        MArrayDataHandle handle = data.inputArrayValue(aCtlWeight);
        if(handle.elementCount() != numPrb){
            MGlobal::displayInfo("# of ctlPts and ctlWeight are different");
            return MS::kFailure;
        }
        for(int i=0;i<numPrb;i++){
            handle.jumpToArrayElement(i);
            ctlWeight[i]=handle.inputValue().asDouble();
        }
        
        
        // weight computation
        D.setNum(numPrb, numPts, mesh.numTet);
        D.computeDistPts(pts, initCtlPoints);
        D.computeDistTet(tetCenter, initCtlPoints);
        D.findClosestTet();
        D.findClosestPts();
        std::vector< std::vector<double> > dist;
        if(poisson){
            dist = D.distTet;
            if(recompARAP){
                // find constraint points
                double constraintWeight = data.inputValue( aConstraintWeight ).asDouble();
                constraint.resize(numPrb);
                for(int i=0;i<numPrb;i++){
                    constraint[i] = T(i,D.closestPts[i],constraintWeight*ctlWeight[i]);
                }
                if( constraintMode == CONSTRAINT_NEIGHBOUR ){
                    double constraintRadius = data.inputValue( aConstraintRadius ).asDouble();
                    for(int i=0;i<numPrb;i++){
                        double r = constraintRadius;
                        for(int j=0;j<numPts;j++){
                            if(D.distPts[i][j]<r){
                                constraint.push_back(T(i,j,constraintWeight * ctlWeight[i] * pow((r-D.distPts[i][j])/r,normExponent)));
                            }
                        }
                    }
                }
                int numConstraint=constraint.size();
                mesh.constraintWeight.resize(numConstraint);
                mesh.constraintVal.resize(numConstraint,numPrb);
                for(int cur=0;cur<numConstraint;cur++){
                    mesh.constraintWeight[cur] = std::make_pair(constraint[cur].col(), constraint[cur].value());
                }
                //
                isError = mesh.ARAPprecompute();
                if(isError>0) return MS::kFailure;
                status = data.setClean(aRecompARAP);
            }
        }else{
            dist = D.distPts;
        }

        w.resize(num);
        for(int j=0;j<num;j++){
            w[j].resize(numPrb);
        }

        if(weightMode == WM_INV_DISTANCE){
            for(int j=0; j<num; j++ ){
                for(int i=0;i<numPrb;i++){
                    w[j][i] = ctlWeight[i] / pow(dist[i][j],normExponent);
                }
            }
        }else if(weightMode == WM_CUTOFF_DISTANCE){
            for(int j=0; j<num; j++ ){
                for( int i=0; i<numPrb; i++){
                    w[j][i] = (dist[i][j] > effectRadius)
                    ? EPSILON : ctlWeight[i]*pow((effectRadius-dist[i][j])/effectRadius,normExponent);
                }
            }
        }else if(weightMode & WM_HARMONIC){
            Laplacian harmonicWeighting;
            makeFaceTet(data, input, inputGeom, mIndex, pts, harmonicWeighting.tetList, harmonicWeighting.tetMatrix, harmonicWeighting.tetWeight);
            harmonicWeighting.numTet = (int)harmonicWeighting.tetList.size()/4;
            std::vector<T> weightConstraint(numPrb);
            // the vertex closest to the probe is given probeWeight
            for(int i=0;i<numPrb;i++){
                weightConstraint[i]=T(i,D.closestPts[i],ctlWeight[i]);
            }
            // vertices within effectRadius are given probeWeight
            if( data.inputValue( aNeighbourWeighting ).asBool() ){
                for(int i=0;i<numPrb;i++){
                    for(int j=0;j<numPts;j++){
                        if(D.distPts[i][j]<effectRadius){
                            weightConstraint.push_back(T(i,j,ctlWeight[i]));
                        }
                    }
                }
            }
            // set boundary condition for weight computation
            int numConstraint=weightConstraint.size();
            harmonicWeighting.constraintWeight.resize(numConstraint);
            harmonicWeighting.constraintVal.resize(numConstraint,numPrb);
            harmonicWeighting.constraintVal.setZero();
            for(int i=0;i<numConstraint;i++){
                harmonicWeighting.constraintVal(i,weightConstraint[i].row())=weightConstraint[i].value();
                harmonicWeighting.constraintWeight[i] = std::make_pair(weightConstraint[i].col(), weightConstraint[i].value());
            }
            // clear tetWeight
            if(!areaWeighted){
                harmonicWeighting.tetWeight.assign(harmonicWeighting.numTet,1.0);
            }
            // solve the laplace equation
            harmonicWeighting.dim = numPts;
            isError = harmonicWeighting.cotanPrecompute();
            if(isError>0) return MS::kFailure;
            harmonicWeighting.harmonicSolve();
            if(poisson){
                std::vector< std::vector<double> > w_tet(numPrb);
                for(int i=0;i<numPrb;i++){
                    makeTetWeightList(tetMode, mesh.tetList, faceList, edgeList, vertexList, harmonicWeighting.Sol.col(i), w_tet[i]);
                    for(int j=0;j<mesh.numTet; j++){
                        w[j][i] = w_tet[i][j];
                    }
                }
            }else{
                for(int i=0;i<numPrb;i++){
                    for(int j=0;j<numPts;j++){
                        w[j][i] = harmonicWeighting.Sol(j,i);
                    }
                }
            }
        }
        // make positive/normalise weights
        if (data.inputValue( aPositiveWeight ).asBool()){
            for(int j=0;j<num;j++){
                for (int i = 0; i < numPrb; i++){
                    w[j][i] = max(w[j][i], 0.0);
                }
            }
        }
        bool normaliseWeight = data.inputValue( aNormaliseWeight ).asBool();
        for(int j=0;j<num;j++){
            double sum = std::accumulate(w[j].begin(), w[j].end(), 0.0);
            if ( (sum > 1 || normaliseWeight) && sum>0){
                for (int i = 0; i < numPrb; i++){
                    w[j][i] /= sum;
                }
            }
        }
        
        // MLS precomputation
        icenter.resize(num);
        p.resize(num); trPP.resize(num); PPI.resize(num);
        for(int j=0; j<num; j++ ){
            // barycentre of the init ctrl points
            icenter[j] = Vector3d::Zero();
            for(int i=0;i<numPrb;i++){
                icenter[j] += w[j][i] * initCtlPoints[i];
            }
            p[j].resize(3, numPrb);
            for(int i=0;i<numPrb;i++){
                p[j].col(i) = w[j][i] * (initCtlPoints[i]-icenter[j]);
            }
            Matrix3d PP = p[j]*p[j].transpose();
            if(PP.determinant()==0){
                isError = ERROR_MLS_SINGULAR;
                MGlobal::displayInfo("ctlPts are coplanar");
                return MS::kFailure;
            }
            trPP[j] = PP.trace();
            PPI[j] = PP.inverse().eval();
        }
        data.setClean(aRecompMLS);
    }
    
    
    // compute deformation
    std::vector<Matrix4d> A(num);
    for(int j=0; j<num; j++ ){
        Vector3d center=Vector3d::Zero();
        for(int i=0;i<numPrb;i++){
            center += w[j][i] * ctlPoints[i];
        }
        // determine the closest affine matrix
        MatrixXd q(3,numPrb);   // relative coordinates of the cage
        Matrix3d M,S;
        for(int i=0;i<numPrb;i++){
            q.col(i) = (w[j][i]) * (ctlPoints[i]-center);
        }
        JacobiSVD<Matrix3d> svd(p[j]*q.transpose(), ComputeFullU | ComputeFullV);
        M = PPI[j] * (p[j]*q.transpose());
        Matrix3d sgn = Matrix3d::Identity();
        if(M.determinant()<0){
            sgn(2,2) = -1;
        }
        if(MLSMode == MLS_SIM){
            M = svd.matrixU() * sgn * svd.matrixV().transpose();
            M *= svd.singularValues().sum()/trPP[j];
        }else if(MLSMode == MLS_RIGID){
            M = svd.matrixU() * sgn * svd.matrixV().transpose();
        }
        A[j] = pad(M, center-icenter[j]);
    }
    if(poisson){
        // set constraint
        int numConstraints = constraint.size();
        mesh.constraintVal.resize(numConstraints,3);
        RowVector3d cv;
        for(int cur=0;cur<numConstraints;cur++){
            cv = pts[constraint[cur].col()] + ctlPoints[constraint[cur].row()]-initCtlPoints[constraint[cur].row()];
            mesh.constraintVal(cur,0) = cv[0];
            mesh.constraintVal(cur,1) = cv[1];
            mesh.constraintVal(cur,2) = cv[2];
        }
        
        // iterate to determine vertices position
        for(int k=0;k<numIter;k++){
            // solve ARAP
            mesh.ARAPSolve(A);
            // set new vertices position
            for(int i=0;i<numPts;i++){
                pts[i][0]=mesh.Sol(i,0);
                pts[i][1]=mesh.Sol(i,1);
                pts[i][2]=mesh.Sol(i,2);
            }
            // if iteration continues
            if(k+1<numIter){
                std::vector<double> dummy_weight;
                makeTetMatrix(tetMode, pts, mesh.tetList, faceList, edgeList, vertexList, Q, dummy_weight);
                Matrix3d S,R,newS,newR;
                for(int i=0;i<mesh.numTet;i++){
                    polarHigham(A[i].block(0,0,3,3), S, R);
                    polarHigham((mesh.tetMatrixInverse[i]*Q[i]).block(0,0,3,3), newS, newR);
                    A[i].block(0,0,3,3) = S*newR;
                }
            }
        }
        for(int j=0;j<numPts;j++){
            Mpts[j].x=mesh.Sol(j,0);
            Mpts[j].y=mesh.Sol(j,1);
            Mpts[j].z=mesh.Sol(j,2);
            Mpts[j] *= localToWorldMatrix.inverse();
        }
    }else{
        for(int j=0; j<numPts; j++ ){
            RowVector4d v = pad(pts[j]) * A[j];
            Mpts[j].x = v[0];
            Mpts[j].y = v[1];
            Mpts[j].z = v[2];
            Mpts[j] *= localToWorldMatrix.inverse();
        }
    }
    
    itGeo.setAllPositions(Mpts);
        
    return MS::kSuccess;
}


// maya plugin initialization
MStatus MLSDeformerNode::initialize(){
    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;
 
    // this attr will be dirtied when MLS recomputation is needed
    aRecompMLS = nAttr.create( "recompMLS", "recompMLS", MFnNumericData::kBoolean, true );
    nAttr.setHidden(true);
    nAttr.setStorable(false);
    nAttr.setKeyable(false);
    addAttribute( aRecompMLS );

    // this attr will be dirtied when ARAP recomputation is needed
    aRecompARAP = nAttr.create( "recompARAP", "recompARAP", MFnNumericData::kBoolean, true );
    nAttr.setHidden(true);
    nAttr.setStorable(false);
    nAttr.setKeyable(false);
    addAttribute( aRecompARAP );
    
    
    aMLSMode = eAttr.create( "MLSMode", "mlsm", MLS_RIGID );
    eAttr.addField( "MLS-AFF", MLS_AFF );
    eAttr.addField( "MLS-SIM", MLS_SIM );
    eAttr.addField( "MLS-RIGID", MLS_RIGID );
    eAttr.addField( "OFF", MLS_OFF );
    addAttribute( aMLSMode );
    attributeAffects( aMLSMode, outputGeom );
    attributeAffects( aMLSMode, aRecompMLS );
    
    aNormaliseWeight = nAttr.create( "normaliseWeight", "nw", MFnNumericData::kBoolean, true );
    nAttr.setStorable(true);
    addAttribute( aNormaliseWeight );
    attributeAffects( aNormaliseWeight, outputGeom );
    attributeAffects( aNormaliseWeight, aRecompMLS );

    aAreaWeighted = nAttr.create( "areaWeighted", "aw", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aAreaWeighted );
    attributeAffects( aAreaWeighted, outputGeom );
    attributeAffects( aAreaWeighted, aRecompMLS );

    aPositiveWeight = nAttr.create( "positiveWeight", "posw", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aPositiveWeight );
    attributeAffects( aPositiveWeight, outputGeom );
    attributeAffects( aPositiveWeight, aRecompMLS  );

    aWeightMode = eAttr.create( "weightMode", "wtm", WM_INV_DISTANCE );
    eAttr.addField( "inverse", WM_INV_DISTANCE );
    eAttr.addField( "cut-off", WM_CUTOFF_DISTANCE );
//    eAttr.addField( "harmonic-arap", WM_HARMONIC );
    eAttr.addField( "harmonic-cotan", WM_HARMONIC_COTAN );
    eAttr.setStorable(true);
    addAttribute( aWeightMode );
    attributeAffects( aWeightMode, outputGeom );
    attributeAffects( aWeightMode, aRecompMLS );

    aEffectRadius = nAttr.create("effectRadius", "er", MFnNumericData::kDouble, 8.0);
    nAttr.setMin( EPSILON );
    nAttr.setStorable(true);
    addAttribute( aEffectRadius );
    attributeAffects( aEffectRadius, outputGeom );
    attributeAffects( aEffectRadius, aRecompMLS );
    
    aNormExponent = nAttr.create("normExponent", "ne", MFnNumericData::kDouble, 2.0);
    nAttr.setStorable(true);
	addAttribute( aNormExponent );
	attributeAffects( aNormExponent, outputGeom );
	attributeAffects( aNormExponent, aRecompMLS );
    
    aNeighbourWeighting = nAttr.create( "neighbourWeighting", "nghbrw", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aNeighbourWeighting );
    attributeAffects( aNeighbourWeighting, outputGeom );
    attributeAffects( aNeighbourWeighting, aRecompMLS );
    
    aPoisson = nAttr.create( "poisson", "poisson", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aPoisson );
    attributeAffects( aPoisson, outputGeom );
    attributeAffects( aPoisson, aRecompARAP );
    attributeAffects( aPoisson, aRecompMLS );

    aCtlWeight = nAttr.create("ctlWeight", "ctlw", MFnNumericData::kDouble, 1.0);
    nAttr.setArray(true);
    nAttr.setStorable(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aCtlWeight);
    attributeAffects( aCtlWeight, outputGeom );
    attributeAffects( aCtlWeight, aRecompMLS);

    aCtlPoints = nAttr.create("ctlPoints", "cp", MFnNumericData::k3Double);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aCtlPoints);
    attributeAffects( aCtlPoints, outputGeom);

    aInitCtlPoints = nAttr.create("initCtlPoints", "icp", MFnNumericData::k3Double);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aInitCtlPoints);
    attributeAffects( aInitCtlPoints, outputGeom);
    attributeAffects( aInitCtlPoints, aRecompMLS);
    attributeAffects( aInitCtlPoints, aRecompARAP );
    
    
    aTetMode = eAttr.create( "tetMode", "tm", TM_FACE);
    eAttr.addField( "face", TM_FACE );
    eAttr.addField( "edge", TM_EDGE );
    eAttr.addField( "vertex", TM_VERTEX );
    eAttr.addField( "vface", TM_VFACE );
    eAttr.setStorable(true);
    addAttribute( aTetMode );
    attributeAffects( aTetMode, outputGeom );
    attributeAffects( aTetMode, aRecompARAP );
    attributeAffects( aTetMode, aRecompMLS );

    aIteration = nAttr.create("iteration", "it", MFnNumericData::kShort, 1);
    nAttr.setStorable(true);
    addAttribute(aIteration);
    attributeAffects(aIteration, outputGeom);
    
    aConstraintMode = eAttr.create( "constraintMode", "ctm", CONSTRAINT_CLOSEST );
    eAttr.addField( "neighbour",  CONSTRAINT_NEIGHBOUR);
    eAttr.addField( "closestPt", CONSTRAINT_CLOSEST );
    eAttr.setStorable(true);
    addAttribute( aConstraintMode );
    attributeAffects( aConstraintMode, outputGeom );
    attributeAffects( aConstraintMode, aRecompMLS);
    attributeAffects( aConstraintMode, aRecompARAP );
    
    aConstraintWeight = nAttr.create("constraintWeight", "cw", MFnNumericData::kDouble, 1e-10);
    nAttr.setStorable(true);
    addAttribute( aConstraintWeight );
    attributeAffects( aConstraintWeight, outputGeom );
    attributeAffects( aConstraintWeight, aRecompMLS);
    attributeAffects( aConstraintWeight, aRecompARAP );
    
    aConstraintRadius = nAttr.create("constraintRadius", "cr", MFnNumericData::kDouble, 2.0);
    nAttr.setStorable(true);
    addAttribute( aConstraintRadius );
    attributeAffects( aConstraintRadius, outputGeom );
    attributeAffects( aConstraintRadius, aRecompMLS );
    attributeAffects( aConstraintRadius, aRecompARAP );

    return MS::kSuccess;
}
 
MStatus initializePlugin( MObject obj )
{
    MStatus status;
    MFnPlugin plugin( obj, "Shizuo KAJI", "0.1", "Any");
 
    status = plugin.registerNode( MLSDeformerNode::nodeName, MLSDeformerNode::id, MLSDeformerNode::creator, MLSDeformerNode::initialize, MPxNode::kDeformerNode );
    CHECK_MSTATUS_AND_RETURN_IT( status );
 
    return status;
}
 
MStatus uninitializePlugin( MObject obj )
{
    MStatus   status;
    MFnPlugin plugin( obj );
 
    status = plugin.deregisterNode( MLSDeformerNode::id );
    CHECK_MSTATUS_AND_RETURN_IT( status );
 
    return status;
}
