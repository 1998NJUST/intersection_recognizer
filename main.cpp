#include<iostream>
#include "intersection_recognizer.h"
using namespace std;
using namespace cv;

int main(int argc, char**argv)
{
    
    std::string path = argv[1];
    IntersectionRecognizer intersectionrecognizer;
    intersectionrecognizer.load_img(path);
    return 0;
}