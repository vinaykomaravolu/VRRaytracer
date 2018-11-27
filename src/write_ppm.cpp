#include "../include/write_ppm.h"
#include <fstream>
#include <cassert>
#include <iostream>

using namespace std;

bool write_ppm(
  const std::string & filename,
  const std::vector<unsigned char> & data,
  const int width,
  const int height,
  const int num_channels)
{
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code from computer-graphics-raster-images or email
  // jacobson@cs.toronto.edu for correct code.
  assert(
    (num_channels == 3 || num_channels == 1 ) &&
    ".ppm only supports RGB or grayscale images");
  ////////////////////////////////////////////////////////////////////////////
  //Open file
  std::ofstream file;
  file.open(filename.c_str());

  //Check to see if file is opened
  if(!file.is_open()){
    return false;
  }

  file << "P3" << std::endl;
  file << width << " " << height << std::endl;
  file << "255" << std::endl;
  //RGB WRITE
  if(num_channels == 3){
    for(int row = 0; row < height; row++){
      for(int column = 0; column < width; column++){
        file << int(data[0 + 3*(column + row * width)]) << " " <<  int(data[1 + 3*(column + row * width)]) << " "  << int(data[2 + 3*(column + row * width)]) << std::endl;
      }
    }
    return true;
  }else if(num_channels == 1){
    for(int row = 0; row < height; row++){
      for(int column = 0; column < width; column++){
        file << int(data[(column + row * width)]) << " " <<  int(data[(column + row * width)]) << " "  << int(data[(column + row * width)]) << std::endl;
      }
    }
    return true;
  }
  return false;
  ////////////////////////////////////////////////////////////////////////////
}
