#include "intersection_recognizer.h"


bool compareNumeric(const String &a, const String &b)
{
    regex re("(\\d+)");
    smatch match_a, match_b;

    string name_a = a.substr(a.find_last_of("/\\") + 1);
    string name_b = b.substr(b.find_last_of("/\\") + 1);

    if (regex_search(name_a, match_a, re) && regex_search(name_b, match_b, re))
    {
        int num_a = stoi(match_a.str(1));
        int num_b = stoi(match_b.str(1));
        return num_a < num_b;
    }
    return a < b;
}

IntersectionRecognizer::IntersectionRecognizer(/* args */) 
{
   preprocess_img_ = cv::Mat::zeros(448, 448, CV_8UC1);
}

void IntersectionRecognizer::preprocess(const cv::Mat& raw_parsing,
                                        cv::Mat& binary_img) {
  int parking_line_pixel_cnt = 0;
  int guide_line_pixel_cnt = 0;
  for (size_t ci = 0; ci < raw_parsing.cols; ci += 1) {
    for (size_t ri = 0; ri < raw_parsing.rows; ri += 1) {
      auto& pixel = raw_parsing.at<uchar>(ri, ci);
      if (pixel == SpLabels20::ROAD) {
        binary_img.at<uchar>(ri, ci) = 255;
      } else if (pixel == SpLabels20::LANE_LINE) {
        binary_img.at<uchar>(ri, ci) = 125;
      } else if (pixel == SpLabels20::PARKING_LINE) {
        binary_img.at<uchar>(ri, ci) = 125;
        parking_line_pixel_cnt++;
      } else if (pixel == SpLabels20::GENERALIZED_CURB) {
        binary_img.at<uchar>(ri, ci) = 255;
      } else if(pixel == SpLabels20::STOP_LINE) {
        binary_img.at<uchar>(ri, ci) = 255;
      }else if (pixel == SpLabels20::CROSS_WALK_LINE) {
        binary_img.at<uchar>(ri, ci) = 255;
      } else if (pixel == SpLabels20::NO_PARKING_SIGN_LINE) {
        binary_img.at<uchar>(ri, ci) = 255;
      } else if (pixel == SpLabels20::ARROW) {
        binary_img.at<uchar>(ri, ci) = 255;
      } else if (pixel == SpLabels20::SPEED_BUMP) {
        binary_img.at<uchar>(ri, ci) = 255;
      } else if (pixel == SpLabels20::GUIDE_LINE) {
        binary_img.at<uchar>(ri, ci) = 255;
        guide_line_pixel_cnt++;
      } else {
        binary_img.at<uchar>(ri, ci) = 0;
      }
    }
  }

  // if (parking_line_pixel_cnt < 3e2) {
  //   return false;
  // }

  // std::cout << "parking_line_pixel_cnt:" << parking_line_pixel_cnt << std::endl;

  // return true;
}



void IntersectionRecognizer::load_img(const string &dir)
{
    vector<cv::String> png_files;
    glob(dir + "/*.png", png_files, false);
    sort(png_files.begin(), png_files.end(), compareNumeric);
    for (const auto &file_path : png_files)
    {
        Mat mat1 = imread(file_path, 0);
        preprocess(mat1, preprocess_img_);
        IntersectionType intersectiontype;
        locateIntersection(preprocess_img_,intersectiontype);
        cout<<"该照片的路径为"<<file_path<<endl;
    }
}


void IntersectionRecognizer::locateIntersection(
    const cv::Mat& preprocessed_img, IntersectionType& intersection_type) {
  std::shared_ptr<PixelLine> linePtr = nullptr;
  std::vector<std::shared_ptr<PixelLine>> pixel_lines;
  std::vector<std::shared_ptr<PixelLine>> lines_tmp;
  bool new_line = false;
  bool is_last_target_pixel = false;
  cv::imshow("原始图",preprocessed_img);
  // 第一行
  for (size_t ci = 0; ci < preprocessed_img.cols; ci++) {
    auto& pixel = preprocessed_img.at<uchar>(0, ci);
    bool is_target_pixel = pixel > 100;

    if (!is_last_target_pixel && is_target_pixel) {
      new_line = true;
      linePtr = std::make_shared<PixelLine>();
      linePtr->start.x = ci;
      linePtr->start.y = 0;
      linePtr->id = line_id_++;
      lines_tmp.emplace_back(linePtr);
    } else if (new_line && ci == preprocessed_img.cols - 1 && is_target_pixel &&
               is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = ci;
      linePtr->end.y = 0;
      linePtr->computeLineLength();
    } else if (new_line && ci > 0 && !is_target_pixel && is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = ci - 1;
      linePtr->end.y = 0;
      linePtr->computeLineLength();
    }

    is_last_target_pixel = is_target_pixel;
  }

  if (lines_tmp.size() >= 2) {
    auto line_it =
        std::max_element(lines_tmp.begin(), lines_tmp.end(),
                         [&](const std::shared_ptr<PixelLine>& line_0,
                             const std::shared_ptr<PixelLine>& line_1) -> bool {
                           return line_0->length < line_1->length;
                         });
    pixel_lines.emplace_back(*line_it);
  } else {
    pixel_lines.insert(pixel_lines.end(), lines_tmp.begin(), lines_tmp.end());
  }

  // 最后一列
  new_line = false;
  is_last_target_pixel = false;
  lines_tmp.clear();
  for (size_t ri = 0; ri < preprocessed_img.rows; ri++) {
    auto& pixel = preprocessed_img.at<uchar>(ri, preprocessed_img.cols - 1);
    bool is_target_pixel = pixel > 100;

    if (!is_last_target_pixel && is_target_pixel) {
      new_line = true;
      linePtr = std::make_shared<PixelLine>();
      linePtr->start.x = preprocessed_img.cols - 1;
      linePtr->start.y = ri;
      linePtr->id = line_id_++;
      lines_tmp.emplace_back(linePtr);
    } else if (new_line && ri == preprocessed_img.rows - 1 && is_target_pixel &&
               is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = preprocessed_img.cols - 1;
      linePtr->end.y = ri;
      linePtr->computeLineLength();
    } else if (new_line && ri > 0 && !is_target_pixel && is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = preprocessed_img.cols - 1;
      linePtr->end.y = ri - 1;
      linePtr->computeLineLength();
    }

    is_last_target_pixel = is_target_pixel;
  }
  if (lines_tmp.size() >= 2) {
    auto line_it =
        std::max_element(lines_tmp.begin(), lines_tmp.end(),
                         [&](const std::shared_ptr<PixelLine>& line_0,
                             const std::shared_ptr<PixelLine>& line_1) -> bool {
                           return line_0->length < line_1->length;
                         });
    pixel_lines.emplace_back(*line_it);
  } else {
    pixel_lines.insert(pixel_lines.end(), lines_tmp.begin(), lines_tmp.end());
  }

  // 最后一行
  new_line = false;
  is_last_target_pixel = false;
  lines_tmp.clear();
  for (size_t ci = 0; ci < preprocessed_img.cols; ci++) {
    auto& pixel = preprocessed_img.at<uchar>(preprocessed_img.rows - 1, ci);
    bool is_target_pixel = pixel > 100;

    if (!is_last_target_pixel && is_target_pixel) {
      new_line = true;
      linePtr = std::make_shared<PixelLine>();
      linePtr->start.x = ci;
      linePtr->start.y = preprocessed_img.rows - 1;
      linePtr->id = line_id_++;
      lines_tmp.emplace_back(linePtr);
    } else if (ci == preprocessed_img.cols - 1 && is_target_pixel &&
               is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = ci;
      linePtr->end.y = preprocessed_img.rows - 1;
      std::swap(linePtr->end.x, linePtr->start.x);
      std::swap(linePtr->end.y, linePtr->start.y);
      linePtr->computeLineLength();
    } else if (new_line && ci > 0 && !is_target_pixel && is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = ci - 1;
      linePtr->end.y = preprocessed_img.rows - 1;
      // swap start and end
      std::swap(linePtr->end.x, linePtr->start.x);
      std::swap(linePtr->end.y, linePtr->start.y);
      linePtr->computeLineLength();
    }

    is_last_target_pixel = is_target_pixel;
  }
  if (lines_tmp.size() >= 2) {
    auto line_it =
        std::max_element(lines_tmp.begin(), lines_tmp.end(),
                         [&](const std::shared_ptr<PixelLine>& line_0,
                             const std::shared_ptr<PixelLine>& line_1) -> bool {
                           return line_0->length < line_1->length;
                         });
    pixel_lines.emplace_back(*line_it);
  } else {
    pixel_lines.insert(pixel_lines.end(), lines_tmp.begin(), lines_tmp.end());
  }

  // 第一列
  new_line = false;
  is_last_target_pixel = false;
  lines_tmp.clear();
  for (size_t ri = 0; ri < preprocessed_img.rows; ri++) {
    auto& pixel = preprocessed_img.at<uchar>(ri, 0);
    bool is_target_pixel = pixel > 100;

    if (!is_last_target_pixel && is_target_pixel) {
      new_line = true;
      linePtr = std::make_shared<PixelLine>();
      linePtr->start.x = 0;
      linePtr->start.y = ri;
      linePtr->id = line_id_++;
      lines_tmp.emplace_back(linePtr);
    } else if (ri == preprocessed_img.rows - 1 && is_target_pixel &&
               is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = 0;
      linePtr->end.y = ri;
      // swap start and end
      std::swap(linePtr->end.x, linePtr->start.x);
      std::swap(linePtr->end.y, linePtr->start.y);
      linePtr->computeLineLength();
    } else if (new_line && ri > 0 && !is_target_pixel && is_last_target_pixel) {
      new_line = false;
      linePtr->end.x = 0;
      linePtr->end.y = ri - 1;
      // swap start and end
      std::swap(linePtr->end.x, linePtr->start.x);
      std::swap(linePtr->end.y, linePtr->start.y);
      linePtr->computeLineLength();
    }

    is_last_target_pixel = is_target_pixel;
  }
  if (lines_tmp.size() >= 2) {
    auto line_it =
        std::max_element(lines_tmp.begin(), lines_tmp.end(),
                         [&](const std::shared_ptr<PixelLine>& line_0,
                             const std::shared_ptr<PixelLine>& line_1) -> bool {
                           return line_0->length < line_1->length;
                         });
    pixel_lines.emplace_back(*line_it);
  } else {
    pixel_lines.insert(pixel_lines.end(), lines_tmp.begin(), lines_tmp.end());
  }

#if DEBUG_INTERSECTION_RECONGNIZER
  cout << " pixel_lines.size(): " << pixel_lines.size() << endl;
#endif

  for (auto it = pixel_lines.begin(); it != pixel_lines.end();) {
    int diff =
        abs((*it)->start.x - (*it)->end.x) + abs((*it)->start.y - (*it)->end.y);
    (*it)->length = diff + 1;
    if (diff <= 96) {  //车的宽度
      it = pixel_lines.erase(it);
    } else if (diff > 230) {  // 道路的宽度
      it = pixel_lines.erase(it);
    } else {
      it++;
    }
  }

#if DEBUG_INTERSECTION_RECONGNIZER
  cout << " pixel_lines.size(): " << pixel_lines.size() << endl;
#endif

  cv::Point2i image_center(preprocessed_img.rows / 2,
                           preprocessed_img.cols / 2);

#if DEBUG_INTERSECTION_RECONGNIZER
  cv::Mat tmp_img = cv::Mat::zeros(preprocessed_img.size(), CV_8UC1);
#endif
  // 剔除非道路
  std::vector<std::shared_ptr<PixelLine>> valid_lines;
  for (int i = 0; i < pixel_lines.size(); i++) {
    bool valid_line = true;

#if DEBUG_INTERSECTION_RECONGNIZER
    cv::line(tmp_img, pixel_lines[i]->start, pixel_lines[i]->end,
             cv::Scalar(255), 10);
#endif
    // std::cout << "length:" << pixel_lines[i]->length << std::endl;

    std::vector<cv::Point2i> line_points;
    int idx_0 = i;
    int idx_1 = (i + 1) % pixel_lines.size();
    cv::Point2i center_point_0(
        (pixel_lines[idx_0]->start.x + pixel_lines[idx_0]->end.x) / 2.,
        (pixel_lines[idx_0]->start.y + pixel_lines[idx_0]->end.y) / 2.);
    cv::Point2i center_point_1(
        (pixel_lines[idx_1]->start.x + pixel_lines[idx_1]->end.x) / 2.,
        (pixel_lines[idx_1]->start.y + pixel_lines[idx_1]->end.y) / 2.);

    bresenham(center_point_0.x, center_point_0.y, image_center.x,
              image_center.y, line_points);
    bresenham(center_point_1.x, center_point_1.y, image_center.x,
              image_center.y, line_points);

    for (auto& point : line_points) {
      auto& pixel = preprocessed_img.at<uchar>(point.y, point.x);
      if (point.x >= 0 && point.x < preprocessed_img.cols && point.y >= 0 &&
          point.y < preprocessed_img.rows) {
        bool is_target_pixel = pixel > 100;
        if (is_target_pixel) {
#if DEBUG_INTERSECTION_RECONGNIZER
          tmp_img.at<uchar>(point.y, point.x) = 255;
#endif
        } else {
          valid_line = false;
          cout << "invalid line:" << i << ",pixel:(" << point.x << ","
               << point.y << ")" << endl;
          break;
        }
      }
    }

    if (valid_line) {
      valid_lines.emplace_back(pixel_lines[i]);
    }
  }

  // 通过线段之间的夹角剔除线段
  bool valid_angle = true;
  for (size_t i = 0; i < valid_lines.size(); i++) {
    int idx_0 = i;
    int idx_1 = (i + 1) % valid_lines.size();
    cv::Point2i center_point_0(
        (valid_lines[idx_0]->start.x + valid_lines[idx_0]->end.x) / 2.,
        (valid_lines[idx_0]->start.y + valid_lines[idx_0]->end.y) / 2.);
    cv::Point2i center_point_1(
        (valid_lines[idx_1]->start.x + valid_lines[idx_1]->end.x) / 2.,
        (valid_lines[idx_1]->start.y + valid_lines[idx_1]->end.y) / 2.);
    float angle = computeAngle(image_center, center_point_0, center_point_1);
    if (angle < angle_threshold_) {
      valid_angle = false;
      break;
    }

#if DEBUG_INTERSECTION_RECONGNIZER
    std::cout << "angle: " << angle << std::endl;
    std::cout << "length: " << valid_lines[idx_0]->length << std::endl;
#endif
  }

  if (valid_angle && valid_lines.size() == 3) {
    intersection_type = IntersectionType::T_Junction;
  } else if (valid_angle && valid_lines.size() == 4) {
    intersection_type = IntersectionType::Crossroad;
  } else {
    intersection_type = IntersectionType::NonIntersection;
  }

  if(intersection_type==IntersectionType::T_Junction)
  {
    std::cout<<"当前路口是丁字路口"<<endl;
  }else if(intersection_type==IntersectionType::Crossroad)
  {
   std::cout<<"当前路口是十字路口"<<endl;
  }else
  {
    std::cout<<"当前没有路口"<<endl;
  }

#if DEBUG_INTERSECTION_RECONGNIZER
  cv::imshow("tmp_img", tmp_img);
  cv::waitKey(0);
#endif
}

float IntersectionRecognizer::computeAngle(cv::Point2i center, cv::Point2i p0,
                                           cv::Point2i p1) {
  cv::Mat v1 = (cv::Mat_<float>(2, 1) << p0.x - center.x, p0.y - center.y);
  cv::Mat v2 = (cv::Mat_<float>(2, 1) << p1.x - center.x, p1.y - center.y);
  float dot_product = v1.dot(v2);

  return acos(dot_product / (cv::norm(v1) * cv::norm(v2))) * RAD2DEG;
}

void IntersectionRecognizer::bresenham(int x1, int y1, int x2, int y2,
                                       std::vector<cv::Point2i>& line_points) {
  // Find Delta
  int dx = x2 - x1;
  int dy = y2 - y1;

  // Find Signs
  int sx = (dx >= 0) ? 1 : (-1);
  int sy = (dy >= 0) ? 1 : (-1);

  // Get Initial Points
  int x = x1;
  int y = y1;

  // Flag to check if swapping happens
  int isSwaped = 0;

  // Swap if needed
  if (abs(dy) > abs(dx)) {
    // swap dx and dy
    int tdx = dx;
    dx = dy;
    dy = tdx;

    isSwaped = 1;
  }

  // Decision parameter
  int p = 2 * (abs(dy)) - abs(dx);

  // Print Initial Point
  // putpixels(x, y);
  line_points.emplace_back(cv::Point2i(x, y));

  // Loop for dx times
  for (int i = 0; i <= abs(dx); i++) {
    // Depending on decision parameter
    if (p < 0) {
      if (isSwaped == 0) {
        x = x + sx;
        // putpixels(x, y);
        line_points.emplace_back(cv::Point2i(x, y));
      } else {
        y = y + sy;
        // putpixels(x, y);
        line_points.emplace_back(cv::Point2i(x, y));
      }
      p = p + 2 * abs(dy);
    } else {
      x = x + sx;
      y = y + sy;
      // putpixels(x, y);
      line_points.emplace_back(cv::Point2i(x, y));
      p = p + 2 * abs(dy) - 2 * abs(dx);
    }
  }
}