#include "ros/ros.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>

//https://gitlab.tu-berlin.de/breakdowncookie/ORB_SLAM2/blob/987e93d81ce3c837be23990fcf62f1dc539efc70/orb_slam2_ros/src/ROSPublisher.cpp
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <octomap_msgs/GetOctomap.h>



using namespace std;
using namespace octomap;

void print_query_info(point3d query, OcTreeNode *node)
{
  if (node != NULL)
  {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

int main(int argc, char **argv)
{
  cout << endl;
  cout << "generating example map" << endl;

  octomap::OcTree tree(0.05); // create empty tree with resolution 0.1
                    // insert some measurements of occupied cells

  for (int x = -1; x < 2; x++)
  {
    for (int y = -1; y < 1; y++)
    {
      for (int z = -1; z < 1; z++)
      {
        point3d endpoint((float)x * 0.05f, (float)y * 0.05f, (float)z * 0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells
/*
  for (int x = -30; x < 30; x++)
  {
    for (int y = -30; y < 30; y++)
    {
      for (int z = -30; z < 30; z++)
      {
        point3d endpoint((float)x * 0.02f - 1.0f, (float)y * 0.02f - 1.0f, (float)z * 0.02f - 1.0f);
        tree.updateNode(endpoint, false); // integrate 'free' measurement
      }
    }
  }
  

  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query(0., 0., 0.);
  OcTreeNode *result = tree.search(query);
  print_query_info(query, result);

  query = point3d(-1., -1., -1.);
  result = tree.search(query);
  print_query_info(query, result);

  query = point3d(1., 1., 1.);
  result = tree.search(query);
  print_query_info(query, result);

  cout << endl;
  //tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt" << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl << endl;

    cout<<"Conversion successfull";
*/
  ros::init(argc, argv, "octomap_trial");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  ros::Publisher pub_octomap = n.advertise<octomap_msgs::Octomap>("octomap_loaded", 1);
  octomap_msgs::Octomap octomap;
    cout<<"Conversion successfull";

  OcTree myOctomap("simple_tree.bt");
  if(octomap_msgs::binaryMapToMsg(tree, octomap)|| 1==1){
    cout<<"Conversion successfull";
    while (ros::ok())
	  {
      //rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
      octomap.header.frame_id = "map";
      octomap.header.stamp = ros::Time::now();
      pub_octomap.publish(octomap);
      ros::spinOnce();
		  loop_rate.sleep();
    }
  }
}
