//
//  Header.h
//  example
//
//  Created by mac on 19-06-30.
//

#ifndef Header_h
#define Header_h
/*
 ************ Exercice (1) : Ajout interactif et affichage des points sur la courbe (50 %)
 */

// Calculate and retutrn the collision position using the baricentric position "bc" and the face "faceID"
Eigen::Vector3f CollisionPositionBC( Eigen::Vector3f bc,int faceID,Eigen::MatrixXd V, Eigen::MatrixXi F)
{
    Eigen::Vector3f position;
    position(0)=bc(0) * V(F(faceID,0),0) + bc(1) * V(F(faceID,1),0)+ bc(2) * V(F(faceID,2),0);
    position(1)=bc(0) * V(F(faceID,0),1) + bc(1) * V(F(faceID,1),1)+ bc(2) * V(F(faceID,2),1);
    position(2)= 0.01;
    return position ;
}
// return the rotated matrix of points around axis X angle dgree
Eigen::MatrixXd rotatePoints(Eigen::MatrixXd points, double angle)
{
    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::TransformEigen::Transform<double, 3, Eigen::Affine>::Identity();
    t.rotate( Eigen::AngleAxisd( angle / 180 * M_PI, Eigen::Vector3d::UnitX() ) );
    
    Eigen::MatrixXd V_transformed = t.matrix() * (Eigen::Map<Eigen::MatrixXdEigen::MatrixXd>(points.data(), points.rows(), points.cols()).rowwise().homogeneous().transpose());
    V_transformed.transposeInPlace();
    
    V_transformed.conservativeResize(V_transformed.rows(), 3);
    return V_transformed ;
}

/*
 * Useful methodes
 */
double randd() {
    return (double)rand() / (RAND_MAX + 1.0);
}

#endif /* Header_h */
