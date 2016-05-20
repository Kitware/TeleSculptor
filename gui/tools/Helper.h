#include <algorithm>
#include <vector>
#include <sstream>
#include <string>

// VTK includes
#include <vtksys/SystemTools.hxx>
#include "vtkMatrix3x3.h"
#include "vtkMatrix4x4.h"
#include "vtkNew.h"
#include "vtkTransform.h"

namespace help
{
  //----------------------------------------------------------------------------
  // Description
  // Split string by separated char
  static void SplitString(const std::string &s, char delim,
                          std::vector<std::string> &elems)
  {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
      {
      elems.push_back(item);
      }
  }

  //----------------------------------------------------------------------------
  // Description
  // Get the filename path (copy from vtk in order to allow debug)
  static std::string GetFilenamePath(const std::string& filename)
  {
    std::string fn = filename;
    vtksys::SystemTools::ConvertToUnixSlashes(fn);

    std::string::size_type slash_pos = fn.rfind("/");
    if (slash_pos != std::string::npos)
    {
      std::string  ret = fn.substr(0, slash_pos);
      if (ret.size() == 2 && ret[1] == ':')
      {
        return ret + '/';
      }
      if (ret.empty())
      {
        return "/";
      }
      return ret;
    }
    else
    {
      return "";
    }
  }

  //----------------------------------------------------------------------------
  // Description
  // Read global path and extract all contained path
  static std::vector<std::string> ExtractAllFilePath(const char* globalPath)
  {
    std::vector<std::string> pathList;

    // Open file which contains the list of all file
    std::ifstream container(globalPath);
    if (!container.is_open())
    {
      std::cerr << "Unable to open : " << globalPath << std::endl;
      return pathList;
    }

    // Extract path of globalPath from globalPath
    //std::string pwd_str = globalPath;
    std::string directoryPath = help::GetFilenamePath(std::string(globalPath));
    // Get current working directory
    if (directoryPath == "")
    {
      directoryPath = vtksys::SystemTools::GetCurrentWorkingDirectory();
    }

    std::string path;
    while (!container.eof())
    {
      std::getline(container, path);
      // only get the file name, not the whole path
      std::vector <std::string> elems;
      help::SplitString(path, ' ', elems);

      // check if there are an empty line
      if (elems.size() == 0)
      {
        continue;
      }

      // Create the real data path to access depth map file
      pathList.push_back(directoryPath + "/" + elems[elems.size() - 1]);
    }

    return pathList;
  }

  //----------------------------------------------------------------------------
  // Description
  // Read krtd file and create K and RT matrix
  static bool ReadKrtdFile(std::string filename, vtkMatrix3x3* matrixK,
                           vtkMatrix4x4* matrixTR)
  {
    // Open the file
    std::ifstream file(filename.c_str());
    if (!file.is_open())
    {
      std::cerr << "Unable to open krtd file : " << filename << std::endl;
      return false;
    }

    std::string line;

    // Get matrix K
    for (int i = 0; i < 3; i++)
    {
      getline(file, line);
      std::istringstream iss(line);

      for (int j = 0; j < 3; j++)
      {
        double value;
        iss >> value;
        matrixK->SetElement(i, j, value);
      }
    }

    getline(file, line);

    // Get matrix R
    for (int i = 0; i < 3; i++)
    {
      getline(file, line);
      std::istringstream iss(line);

      for (int j = 0; j < 3; j++)
      {
        double value;
        iss >> value;
        matrixTR->SetElement(i, j, value);
      }
    }

    getline(file, line);

    // Get matrix T
    getline(file, line);
    std::istringstream iss(line);
    for (int i = 0; i < 3; i++)
    {
      double value;
      iss >> value;
      matrixTR->SetElement(i, 3, value);
    }

    // Finalize matrix TR
    for (int j = 0; j < 4; j++)
    {
      matrixTR->SetElement(3, j, 0);
    }
    matrixTR->SetElement(3, 3, 1);

    return true;
  }


  //----------------------------------------------------------------------------
  // Description
  // Compute median of a vector
  template <typename T>
  static void ComputeMedian(std::vector<T> vector, double& median)
  {
    std::sort(vector.begin(), vector.end());
    size_t middleIndex = vector.size() / 2;
    if (vector.size() % 2 == 0)
      {
      median = (vector[middleIndex] + vector[middleIndex - 1]) / 2;
      }
    else
      {
      median = vector[middleIndex];
      }
  }
}