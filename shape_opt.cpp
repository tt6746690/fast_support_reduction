#include <iostream>
#include <stan/math.hpp>


int main(int argc, char *argv[])
{

    std::cout << "log normal(1 | 2, 3)="
            << stan::math::normal_log(1, 2, 3)
            << std::endl;


    return 0;

}