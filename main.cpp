#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

using namespace std;
using namespace igl;
using namespace Eigen;

namespace {
  const Eigen::MatrixXd V= (Eigen::MatrixXd(4,3)<<
			    0.0,-0.5,-0.01, 
			    0.0, 0.5,-0.01, 
			    1.0,-0.5,-0.01, 
			    1.0, 0.5,-0.01).finished(); 
  const Eigen::MatrixXi F = (Eigen::MatrixXi(2,3)<<
    1,3,4,
    1,4,2).finished().array()-1;

  int nbPoints = 2;
  Eigen::MatrixXi edges;
  Eigen::MatrixXd edgeColors;
  
  Eigen::MatrixXd points = Eigen::MatrixXd(nbPoints,3);
  Eigen::MatrixXd Cp;
  Eigen::MatrixXd Ip;

  const int ShiftModifier = 1;

  // Matrices for the surface of revolution vertices and faces.
  Eigen::MatrixXd RV;
  Eigen::MatrixXi RF;

  // Number of strips of faces around the axis. 
  // If nbSubdiv = 4, then the vertices in the plane are rotated 
  // every 360 / 4 = 90 degrees. 
  // nbSubdiv must be greater or equal to 3.
  int nbSubdiv = 4;

  // After the creation of the surface of revolution, we want to 
  // forbid adding points.
  bool addingPoints = true;
}

/*
 Exercice (1) : Ajout interactif et affichage des points sur la courbe (50 %)
*/
// Calculate and retutrn the collision position using the baricentric position "bc" and the face "faceID"
Eigen::Vector3f CollisionPositionBC( Eigen::Vector3f bc,int faceID)
{
    Eigen::Vector3f position;
    position(0)=bc(0) * V(F(faceID,0),0) + bc(1) * V(F(faceID,1),0)+ bc(2) * V(F(faceID,2),0);
    position(1)=bc(0) * V(F(faceID,0),1) + bc(1) * V(F(faceID,1),1)+ bc(2) * V(F(faceID,2),1);
    position(2)= 0.01;
    return position ;
}



void surfaceOfRevolution(igl::opengl::glfw::Viewer& viewer)
{
  cout << "Computing surface of revolution" << endl;

  // Add your code here...

  // Set the new mesh.
  // Uncomment when you are finished coding the preparation of faces 
  // of the surface of revolution.
  //viewer.data().clear();
  //viewer.data().set_mesh(RV, RF);
  //viewer.data().set_face_based(true);
  //viewer.data().show_lines = true;

  // Keep the code below. It will be used to check your 
  // number of vertices and faces.

  // Display the number of vertices and faces of your surface
  // of revolution surface.
  cout << "RV.rows()" << endl << RV.rows() << endl;
  cout << "RF.rows()" << endl << RF.rows() << endl;

  // Disable addition of points.
  addingPoints = false;
}



bool my_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
  if ((modifier & ShiftModifier)
      && addingPoints)
  {
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
  
    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core.view,
				viewer.core.proj, viewer.core.viewport,
				V, F, fid, bc))
    {
      // Resize the matrix while keeping the data.
      points.conservativeResize(points.rows()+1, 3);
        
        // Add your code here...
       
        // caluculate the collision position and then update the position of the added point
        Vector3f pcollision = CollisionPositionBC(bc, fid) ;
        points.row(points.rows()-1) << pcollision(0), pcollision(1), pcollision(2) ;
        //cout<<"Points:"<<endl<<points<<endl;

        
        // Resize the edge
        edges.conservativeResize(edges.rows()+1, 2);
        // Update edges values
        edges(edges.rows()-2,1) = points.rows()-1;
        edges(edges.rows()-1,0) = points.rows()-1;
        edges(edges.rows()-1,1) = 1;
        //cout<<"Edges:"<<endl<<edges<<endl;
        
      // Offset the new point with respect to the plane by setting its
      // Z value to 0.0.
      points(points.rows() - 2,2) = 0.0;

      viewer.data().set_points(points, Cp);
      viewer.data().set_edges(points, edges, edgeColors);
      
      return true;
    }
    return false;
  }
  return false;
}

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
  if ('3'<= key && key <= '9')
  {
    nbSubdiv = key - '1' + 1;
    cout << "Number of subdivisions = " << nbSubdiv << endl;
    return true;
  }
  if (key == 'R')
  {
    if (points.rows() <= 2)
    {
      cout << "At least 3 points are required." << endl;
      return false;
    }
    surfaceOfRevolution(viewer);
    return true;
  }
  return false;
}

int main(int argc, char *argv[])
{
  const Eigen::MatrixXd V= (Eigen::MatrixXd(4,3)<<
			    0.0,-0.5,-0.001, // (1) left down
			    0.0, 0.5,-0.001, // (2) left up
			    1.0,-0.5,-0.001, // (3) right dowmn
			    1.0, 0.5,-0.001 //  (4) right up
                            ).finished();
  const Eigen::MatrixXi F = (Eigen::MatrixXi(2,3)<<
    1,3,4,
    1,4,2).finished().array()-1;

  points.resize(2, 3);
  points.row(0) << 0, 0, 0;
  points.row(1) << 1, 0, 0;

  edges.resize(1,2);
  edges << 0,1;

  Cp.resize(1,3);
  Cp << 1, 1, 1;
  edgeColors.resize(1,3);
  edgeColors << 1, 0, 0;

  igl::opengl::glfw::Viewer viewer;
  viewer.callback_mouse_down = &my_mouse_down;
  viewer.callback_key_down = &key_down;

  viewer.data().set_mesh(V, F);
  RowVector3d grey = RowVector3d(0.5, 0.5, 0.5);
  viewer.data().set_colors(grey);
  viewer.data().set_face_based(true);
  viewer.data().show_lines = false;
  
  viewer.core.camera_zoom /= 1.5;

  viewer.data().set_points(points, Cp);
  viewer.data().set_edges(points, edges, edgeColors);

  cout << endl
       << "Shift-left-click on the plane to add vertices " << endl
       << "  to define the curve used for the surface of revolution." 
       << endl
       << "Press R,r to compute surface of revolution." 
       << endl;
  
  viewer.launch();
}
