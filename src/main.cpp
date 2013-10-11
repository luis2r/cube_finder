#include "cube_finder/cube_finder.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_filter");
  ros::NodeHandle n;
  cube_finder::CubeFinder cf(n);

  ros::spin();

  return 0;
}
