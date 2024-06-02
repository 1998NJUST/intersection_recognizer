#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <functional>
#include <algorithm> 
#include <regex>
#include <numeric>
#define RAD2DEG (180.0f / M_PI)
using namespace std;
using namespace cv;
#define DEBUG_INTERSECTION_RECONGNIZER (1)
typedef struct PixelLine 
{
  unsigned int id{0};
  cv::Point2i start;
  cv::Point2i end;
  int length{0};

  void computeLineLength() {
    length =
        abs(this->start.x - this->end.x) + abs(this->start.y - this->end.y);
  }

} PixelLine;

enum class IntersectionType {
  NonIntersection = 0,
  T_Junction = 1,
  Crossroad = 2
};

enum SpLabels20 {
  ROAD = 0,
  LANE_LINE = 1,
  PARKING_LINE = 2,
  PARKING_SLOT = 3,
  ARROW = 4,
  GUIDE_LINE = 5,
  CROSS_WALK_LINE = 6,
  NO_PARKING_SIGN_LINE = 7,
  STOP_LINE = 8,
  SPEED_BUMP = 9,
  OTHER = 10,
  PARKING_LOCK_OPEN = 11,
  PARKING_LOCK_CLOSE = 12,
  TRAFFIC_CONE = 13,
  PARKING_ROD = 14,
  CURB = 15,
  CEMENT_COLUMN = 16,
  IMMOVABLE_OBSTACLE = 17,
  MOVABLE_OBSTACLE = 18,
  BACKGROUND = 19,
  SIDE_WALK = 20,
  PAINTED_WALL_ROOT = 21,
  GENERALIZED_CURB = 22

};

class IntersectionRecognizer
{
  public:
    IntersectionRecognizer(/* args */);
    void load_img(const string &dir);
  private:
    
    void preprocess(const cv::Mat &raw_parsing, cv::Mat &binary_img);
    void locateIntersection(const cv::Mat &preprocessed_img,
                          IntersectionType &intersection_type);
    void bresenham(int x1, int y1, int x2, int y2,
                 std::vector<cv::Point2i> &line_points);
    float computeAngle(cv::Point2i center, cv::Point2i p0, cv::Point2i p1);
    cv::Mat preprocess_img_;
    unsigned int line_id_{0};
    const float angle_threshold_{80.f};

};