#include <opencv.hpp>
#include <dirent.h>
#include <iostream>

using namespace cv;

/*
This is just a small program for comparing the depth
images after decompression to the ones before
compression. It just looks at the folders provided
and display the respective corresponding frames
together with the absolute distance. There is
a flickering at the side but that doesn't effect
the area where there is information.
*/

void displayDepth(const string& name, Mat& depthMap) {
    Mat show;
    const float scaleFactor = 0.05f;
    depthMap.convertTo(show, CV_8UC1, scaleFactor);
    imshow(name, show);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "You need to supply path for orginal image folder and the recreated image folder." << std::endl;
    }

	Mat cvMat1 = Mat::zeros(480, 640, CV_16UC1);
	Mat cvMat2 = Mat::zeros(480, 640, CV_16UC1);
	std::string dirname1(argv[1]);
	std::string dirname2(argv[2]);
	
	std::string depthName("depth");

    DIR* dir = opendir(dirname1.c_str());
    if (dir == NULL) {
        std::cout << "Can not read directory." << std::endl;
        return -1;
    }
    std::vector<std::string> entries1;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        std::string entry(ent->d_name);
        if (entry.compare(0, depthName.size(), depthName) != 0) {
	        continue;
	    }
        entries1.push_back(entry);
    }
    closedir(dir);

    dir = opendir(dirname2.c_str());
    if (dir == NULL) {
        std::cout << "Can not read directory." << std::endl;
        return -1;
    }
    std::vector<std::string> entries2;
    while ((ent = readdir(dir)) != NULL) {
        std::string entry(ent->d_name);
        if (entry.compare(0, depthName.size(), depthName) != 0) {
	        continue;
	    }
        entries2.push_back(entry);
    }
    closedir(dir);

    std::sort(entries1.begin(), entries1.end());
    std::sort(entries2.begin(), entries2.end());
    
    for (int i = 0; i < entries2.size(); ++i) {
    	std::string file1 = entries1[i];
    	std::string file2 = entries2[i];
    	std::cout << dirname1 + '/' + file1 << std::endl;
    	std::cout << dirname2 + '/' + file2 << std::endl;
        cvMat1 = imread(dirname1 + '/' + file1, -1);
        cvMat2 = imread(dirname2 + '/' + file2, -1); // should be same name!
        imshow("Original", 16*cvMat1);
        imshow("Recreated", 16*cvMat2);
        Mat diff = 100*(cvMat2 - cvMat1);
        displayDepth("Difference", diff);
        //displayDepth("Second", cvMat2);
        waitKey();
    }
}
