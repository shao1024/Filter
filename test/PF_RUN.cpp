#include "particleFilter_CV.h"
using namespace std;

int main(int argc, char const *argv[])
{
    string filename = "/home/ecs-user/Code/Filter/data/CV_data";
    particleFilter_CV pf(101);
    pf.run(filename);
    return 0;
}