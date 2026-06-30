#include "oden.h"
#include <fstream>
#include <iostream>
#include <iomanip>
int main()
{
    std::vector y_init{0.0, 1.0};

    oden::System system(y_init);

    double omega = 3;

    auto func = [&](double t, const decltype(system) &sys) {
        auto y = sys[0];
        
        return std::vector{-std::pow(omega, 2)*y[0]};
    };

    system.bindStateFunction(func);

    double pi = std::numbers::pi;
    oden::ControlParameters solControl{0, 2*pi, 0.01, 0.001, 1e-4, 1e-3};

    auto solution = system.EulerSolver(solControl, true);

    std::ofstream file("output.csv");

    file << "Time" << "y" << std::endl;
    for (auto &&sol : solution)
    {
        file << sol.time << ",";

        for (auto &&val : sol.variables)
        {
            file << val << ",";
        }

        file << std::endl;
    }
    
}