/*!
 * \file	rail_track
 * \author	Abish Asphandiar <abish@uni-bremen.de> 3020632
 * \date	19-October-2016
 * \brief Initializing the object to publish either image or video based on the extension
 */

#include "rail_track/railtrack_img.hpp"
#include "rail_track/railtrack_vid.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "RailTracker");

  if (argc != 2)
  {
    cout << "Please enter file name" << endl;
    return -1;
  }

  string path =  string(argv[1]);

  if ((path.find(".mp4") != std::string::npos) || (path.find(".avi") != std::string::npos))
    RailTrack_VID vid_obstacle(path);
  else
    RailTrack_IMG img_obstacle(path);

  return 0;
}
