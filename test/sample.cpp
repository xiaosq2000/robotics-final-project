
#include "sample.h"

int main()
{
    Sample sample("../share/sample", "../share/sample/rpy.txt");
    sample.~Sample();
    return 0;
}
