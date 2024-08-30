#include "particleFilter_CV.h"
using namespace std;

int main(int argc, char const *argv[])
{
    string filename = "/root/Shao/PF/my_pf/data/CV_data";
    particleFilter_CV pf(101);
    pf.run(filename);
    return 0;
}