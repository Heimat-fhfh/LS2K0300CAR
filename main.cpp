#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>

#include "AAAdefine.h"

using namespace cv;
using namespace std;

int main() {

    cout << "Hello OpenCV " << CV_VERSION << endl;
    VideoCapture Camera(1);  // 使用默认摄像头，通常是 0 或 -1
    if (!Camera.isOpened()) {
        cerr << "Error: Could not open camera." << endl;
        return -1;
    }
    
    
    return 0;
}
