#include "rgb_encoder.h"
#include "rgb_decoder.h"
#include "depth_encoder.h"
#include "depth_decoder.h"
#include <dirent.h>

using namespace cv;

int main()
{
	Mat cvMat = Mat::zeros(480, 640, CV_16UC1);
	std::string dirname("images");
	std::string filename("vid2.mov");
    DIR* dir = opendir(dirname.c_str());
    if (dir == NULL) {
        std::cout << "Can not read directory." << std::endl;
        return -1;
    }
    std::vector<std::string> entries;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        entries.push_back(std::string(ent->d_name));
    }
    closedir(dir);
    std::sort(entries.begin(), entries.end());
	if (true) {
	    std::string rgbName("depth");
	    DepthEncoder encoder(filename);
	    for (string file : entries) {
	        if (file.compare(0, rgbName.size(), rgbName) == 0) {
	        	cvMat = imread(dirname + '/' + file, -1);
	        	encoder.addFrame(cvMat);
	        }
	    }
	}
	if (false) {
		DepthDecoder decoder(filename);
		const float scaleFactor = 0.05f;
		Mat show;
		while (decoder.getFrame(cvMat)) {
			cvMat.convertTo(show, CV_8UC1, scaleFactor);
			imshow("One frame", show);
			waitKey();
		}
	}

	return 0;
}