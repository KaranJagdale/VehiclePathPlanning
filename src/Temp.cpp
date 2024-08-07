#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <functional>

using namespace std;
using namespace Eigen;

hash<float> hashFloat;

size_t vectorToKey(Vector3f const& vec) 
 {

  size_t seed = vec.size();
  for(auto& i : vec) 
  {
    seed ^= hashFloat(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

int main()
{

    Vector3f a = {18.2, 5, 4.97419};
    Vector3f b = {18.2, 5, 4.97419};
    cout << (vectorToKey(a) == vectorToKey(a)) << endl;
    return 0;
}