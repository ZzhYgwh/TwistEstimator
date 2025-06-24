#ifndef DEBUG_FILE_H
#define DEBUG_FILE_H

#include <iostream>
#include <fstream>

// global variable
struct Ceres_Debug_File
{
public:
  Ceres_Debug_File(const std::string path)
  {
    file_path = path;
    if(!CleanCache())
      std::cerr << "Debug File Initial Fialed!" << std::endl;
  }

  bool CleanCache()
  {
    // debug_file.open(file_path);
    // Open();
    debug_file.open(file_path, std::ios::out | std::ios::app);
    debug_file.clear();
    // debug_file.close();
    // Close();
    debug_file.close();

    return true;
  }

  inline void Open()
  {
    debug_file.open(file_path, std::ios::out | std::ios::app);
  }

  inline void Close()
  {
    debug_file.close();
  }
public:
  std::string file_path;
  std::fstream debug_file;
  long int doppler_len;
  long int flow_len;
  long int bias_len;

}; // struct Ceres_Debug_File

// const std::string debug_ceres_path = "/home/hao/Desktop/ceres_solver.debug";

extern Ceres_Debug_File debug_ceres;
extern bool clear_debug;


extern Ceres_Debug_File debug_ceres_jacobis;

// Ceres_Debug_File debug_ceres(debug_ceres_path);

#endif