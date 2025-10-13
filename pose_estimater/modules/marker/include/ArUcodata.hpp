#ifndef ARUCODATA_HPP
#define ARUCODATA_HPP

#include "../../cal/include/cal_base.hpp"
#include <iostream>   
#include <fstream>
#include <sstream>
#include <string> 
#include <vector>
#include <cstdlib>

std::vector<std::string> split(const std::string& s, char delimiter);
Vec3 getPositionFromCSVById(const std::string& filename, int target_id);
void save_vec3_vectors(const std::string& filename, const std::vector<Vec3>& directions);
std::vector<Vec3> get_aruco_corner_positions(std::vector<int>ids);
#endif