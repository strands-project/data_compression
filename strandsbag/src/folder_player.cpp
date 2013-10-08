#include "folder_player.h"

#include <sensor_msgs/Image.h>
#include <dirent.h>
#include <iostream>
#include <fstream>

bool comparator(const std::pair<std::string, double>& a, const std::pair<std::string, double>& b)
{
    return a.second < b.second; // compare timestamps
}

folder_player::folder_player(ros::NodeHandle& n, ros::Publisher& depth_pub, ros::Publisher& rgb_pub, const std::string& dirname) : n(n), depth_pub(depth_pub), rgb_pub(rgb_pub), dirname(dirname), temp_depth(480, 640, CV_16UC1), temp_rgb(480, 640, CV_8UC3)
{
    DIR* dir = opendir(dirname.c_str()); // open bag folder
    if (dir == NULL) {
        ROS_ERROR("Can not read bag folder.");
    }
    std::string p = "."; // we don't want this
    std::string pp = ".."; // or this
    std::string line;
    double sec;
    double nsec;
    std::ifstream timestamps;
    struct dirent* ent;
    // get the first timestamp of every video and sort them
    while ((ent = readdir(dir)) != NULL) {
        std::string d = ent->d_name;
        if (ent->d_type == DT_DIR && d != p && d != pp) {
            std::string t = dirname + "/" + d + "/time.txt";
            timestamps.open(t.c_str()); // except
            if (!std::getline(timestamps, line)) {
                timestamps.close();
                continue;
            }
            timestamps.close();
            sec = double(atoi(line.substr(10, 10).c_str())); // first seconds timestamp
            nsec = double(atoi(line.substr(21, 10).c_str())); // first nanosec timestamp
            dirs.push_back(std::pair<std::string, double>(d, sec+1e-9f*nsec));
        }
    }
    closedir(dir);
    std::sort(dirs.begin(), dirs.end(), &comparator); // sort folders by first timestamp
    dir_nbr = 0;
    
    shutting_down = false;
    read_new_dir(); // read first folder
    // read first files and set timers for publishing
    ros::Time t = read_depth();
    depth_timer = n.createTimer(t - ros::Time::now(), &folder_player::publish_depth, this, true);
    t = read_rgb();
    rgb_timer = n.createTimer(t - ros::Time::now(), &folder_player::publish_rgb, this, true);
}

// extract all of the timestamps and filenames from the next folder
// maybe all should be indexed directly?
void folder_player::read_new_dir()
{
    if (dir_nbr >= dirs.size()) {
        shutting_down = true;
        return;
    }
    std::ifstream timestamps;
    std::string filename = dirname + "/" + dirs[dir_nbr].first + "/time.txt";
    timestamps.open(filename.c_str());
    std::string line;
    std::string rgb = "rgb";
    std::string rgb_sec;
    std::string rgb_nsec;
    std::string depth_sec;
    std::string depth_nsec;
    while (std::getline(timestamps, line)) {
        if (line.compare(0, rgb.size(), rgb) == 0) { // %2 == 0 would do as well
            rgb_sec = line.substr(10, 10); // seconds timestamp
            rgb_nsec = line.substr(21, 10); // nanosec timestamp
            rgb_secs.push_back(atoi(rgb_sec.c_str()));
            rgb_nsecs.push_back(atoi(rgb_nsec.c_str()));
            rgb_names.push_back(dirs[dir_nbr].first + "/" + line.substr(0, 9));
        }
        else {
            depth_sec = line.substr(12, 10); // seconds timestamp
            depth_nsec = line.substr(23, 10); // nanosec timestamp
            depth_secs.push_back(atoi(depth_sec.c_str()));
            depth_nsecs.push_back(atoi(depth_nsec.c_str()));
            depth_names.push_back(dirs[dir_nbr].first + "/" + line.substr(0, 11));
        }
    }
    timestamps.close();
    ++dir_nbr;
}

// read the latest depth file for later use
ros::Time folder_player::read_depth()
{
    std::string depth_path = dirname + "/" + depth_names.front() + ".tiff";
    temp_depth = cv::imread(depth_path, -1);
    
    std_msgs::Header header_depth;
    header_depth.stamp = ros::Time(depth_secs.front(), depth_nsecs.front());
    
    depth_names.pop_front();
    depth_secs.pop_front();
    depth_nsecs.pop_front();
    
    //std::cout << "ROS time: " << ros::Time::now() << std::endl;
    //std::cout << "Depth time: " << header_depth.stamp << std::endl;
    
    header_depth.frame_id = "/camera_depth_frame"; // should probably be a parameter
    
    image_depth = cv_bridge::CvImage(header_depth, "16UC1", temp_depth);
    
    return header_depth.stamp;
}

// read the latest rgb for later use
ros::Time folder_player::read_rgb()
{
    std::string rgb_path = dirname + "/" + rgb_names.front() + ".png";
    temp_rgb = cv::imread(rgb_path, -1);
    
    std_msgs::Header header_rgb;
    header_rgb.stamp = ros::Time(rgb_secs.front(), rgb_nsecs.front());
    
    rgb_names.pop_front();
    rgb_secs.pop_front();
    rgb_nsecs.pop_front();
    
    //std::cout << "RGB time: " << header_rgb.stamp << std::endl;
    
    header_rgb.frame_id = "/camera_rgb_frame"; // should probably be a parameter
    
    image_rgb = cv_bridge::CvImage(header_rgb, "8UC3", temp_rgb);
    
    return header_rgb.stamp;
}

// called by timerevent, published loaded depth and sets the next timer
void folder_player::publish_depth(const ros::TimerEvent& e)
{
    depth_pub.publish(image_depth.toImageMsg()); // publish
    if (depth_names.size() < 10) {
        if (!shutting_down) {
            read_new_dir();
        }
    }
    if (depth_names.empty()) {
        return;
    }
    ros::Time t = read_depth();
    // set a new timer for the next image timestamp
    depth_timer = n.createTimer(t - ros::Time::now(), &folder_player::publish_depth, this, true);
}

// called by timerevent, published loaded rgb and sets the next timer
void folder_player::publish_rgb(const ros::TimerEvent& e)
{
    rgb_pub.publish(image_rgb.toImageMsg()); // publish
    if (rgb_names.size() < 10) {
        if (!shutting_down) {
            read_new_dir();
        }
    }
    if (rgb_names.empty()) {
        return;
    }
    ros::Time t = read_rgb();
    // set a new timer for the next image timestamp
    rgb_timer = n.createTimer(t - ros::Time::now(), &folder_player::publish_rgb, this, true);
}
