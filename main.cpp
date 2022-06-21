#include "Equi2Rect.hpp"
#include <iostream>
#include <stdio.h>
#include <time.h>

#define LOG(msg) std::cout << msg << std::endl

int main(int argc, const char **argv)
{

    clock_t tStart = clock();

    Equi2Rect equi2rect;

    equi2rect.save_rectlinear_image();

    printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

    equi2rect.show_rectlinear_image();

    return 0;
}