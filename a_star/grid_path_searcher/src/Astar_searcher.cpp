#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  // Initialize global bounds
  gl_xl = global_xyz_l.x();
  gl_yl = global_xyz_l.y();
  gl_zl = global_xyz_l.z();

  gl_xu = global_xyz_u.x();
  gl_yu = global_xyz_u.y();
  gl_zu = global_xyz_u.z();

  // Initialize grid sizes
  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  // Set resolution
  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  // Allocate and initialize data
  data = new uint8_t[GLXYZ_SIZE]();

  // Allocate GridNodeMap
  GridNodeMap = new GridNodePtr**[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; ++i) {
    GridNodeMap[i] = new GridNodePtr*[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; ++j) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; ++k) {
        auto tmpIdx = Vector3i(i, j, k);
        auto pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        if (GridNodeMap[i][j][k]->id !=
            0)  // visualize all nodes in open and close list
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_INFO("visited_nodes size : %ld", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i& index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d& pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d& coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i& index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i& index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int& idx_x, const int& idx_y,
                                        const int& idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int& idx_x, const int& idx_y,
                                    const int& idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr>& neighborPtrSets,
                                          vector<double>& edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  if (currentPtr == nullptr) ROS_ERROR("Error: Current pointer is null!");

  /********************************************************
   * STEP 4: finish AstarPathFinder::AstarGetSucc          *
   * This function is used to get the successors of the    *
   * current node.                                         *
   * Idea: index -> coordinate -> edgecost                 *
   * Note: Be careful with the index bound                 *
   *********************************************************/
  // TODO: Implement Step 4
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  /*********************************************************
   * STEP 1: finish the AstarPathFinder::getHeu             *
   * This function is the heuristic function, which is      *
   * used to estimate the cost from the current node to the *
   * goal node.                                             *
   * You can choose one of the following heuristics:        *
   * 1. Manhattan distance                                  *
   * 2. Euclidean distance                                  *
   * 3. Diagonal distance                                   *
   * 4. Dijkstra's algorithm (0)                            *
   * 5. Your own heuristic function                         *
   * Note: You can also compare the influence of different  *
   * heuristics on the path planning.                       *
   * ********************************************************/

  double h;
  auto node1_coord = node1->coord;
  auto node2_coord = node2->coord;

  // TODO: Implement heuristic function

  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr);

  // STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic
  // function
  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /****************************************************************
   * STEP 2: some else preparatory works which should be done      *
   * before while loop                                             *
   * 1. Assign g(xs) = 0, g(n) = inf(already done in initialzation)*
   * 2. Mark start point as visited(expanded) (id 0: no operation, *
   * id: 1 in OPEN, id -1: in CLOSE )                              *
   * **************************************************************/
  // TODO: Make start point as visited

  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;
  Eigen::Vector3i current_idx;  // record the current index

  // this is the main loop
  while (!openSet.empty()) {
    /*************************************************************
     * STEP 3: Remove the node with lowest cost function from open*
     * set to closed set                                          *
     * Note: This part you should use the C++ STL: multimap       *
     * ***********************************************************/
    // TODO: Implement Step 3

    // TODO: If the current node is the goal node, break the loop

    // Get the succetion
    AstarGetSucc(currentPtr, neighborPtrSets,
                 edgeCostSets);  // STEP 4: finish AstarPathFinder::AstarGetSucc

    /***************************************************************
     * STEP 5: For all unexpanded neigbors "m" of node "n", please  *
     * finish this for loop                                         *
     * **************************************************************/
    for (int i = 0; i < (int)neighborPtrSets.size(); i++) {
      // Judge if the neigbors have been expanded
      // Note: neighborPtrSets[i]->id = -1 : expanded, equal to this node is in
      // close set
      //       neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in
      //       open set

      neighborPtr = neighborPtrSets[i];
      if (neighborPtr->id == 0) {  // discover a new node, which is not in the
                                   // closed set and open set
        /**************************************************************
         * STEP 6: As for a new node, do what you need do ,and then put*
         * neighbor in open set and record it                          *
         * Note: shall update: gScore, fScore, cameFrom, id            *
         * **************************************************************/
        // TODO: Implement Step 6

        continue;
      } else if (neighborPtr->id ==
                 1) {  // this node is in open set and need to judge if it needs
                       // to update, the "0" should be deleted when you are
                       // coding
        /**************************************************************
         * STEP 7: As for a node in open set, update it , maintain the *
         * openset ,and then put neighbor in open set and record it    *
         * Note: shall update: gScore; fScore; cameFrom                *
         * **************************************************************/
        // TODO: Implement Step 7

        continue;
      }
    }
  }
  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;
  /****************************************************************
   * STEP 8: trace back from the curretnt nodePtr to get all nodes *
   * along the path                                                *
   * **************************************************************/
  auto ptr = terminatePtr;
  while (ptr->cameFrom != NULL) {
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
  }

  for (auto ptr : gridPath) path.push_back(ptr->coord);

  reverse(path.begin(), path.end());

  return path;
}