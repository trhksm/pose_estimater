#ifndef ARUCODATA_HPP
#define ARUCODATA_HPP

#include "../../cal/include/cal_base.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <opencv2/core.hpp>

std::vector<std::string> split(const std::string& s, char delimiter);
Vec3 getPositionFromCSVById(const std::string& filename, int target_id);
void save_vec3_vectors(const std::string& filename, const std::vector<Vec3>& directions);
std::vector<std::vector<Vec3>> get_aruco_corners_positions(const std::vector<int>& ids);
std::vector<std::vector<int>> get_pairs_id_and_index(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners);
std::vector<std::pair<std::vector<Vec3>,int>> get_pairs_aruco_corners_positions_and_index(const std::vector<std::vector<int>>& pairs_id_and_index);
#endif