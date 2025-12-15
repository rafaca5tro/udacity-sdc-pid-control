#include <iostream>
#include <cassert>
#include <cmath>
#include "PID.h"

int main() {
    // Initialize PID controller with P=1, I=0, D=0
    PID pid(1.0, 0.0, 0.0);

    // Check initial state
    assert(pid.TotalError() == 0.0);

    // Test Step 1
    // CTE = 1.0
    // P_error = 1.0
    // I_error = 1.0
    // D_error = 1.0 - 0.0 = 1.0
    // TotalError = 1.0*1.0 + 1.0*0.0 + 1.0*0.0 = 1.0
    pid.UpdateError(1.0);
    std::cout << "Test 1: CTE=1.0, TotalError=" << pid.TotalError() << std::endl;
    assert(std::abs(pid.TotalError() - 1.0) < 1e-6);

    // Test Step 2
    // CTE = 0.5
    // P_error = 0.5
    // I_error = 1.0 + 0.5 = 1.5
    // D_error = 0.5 - 1.0 = -0.5
    // TotalError = 0.5*1.0 + 1.5*0.0 + (-0.5)*0.0 = 0.5
    pid.UpdateError(0.5);
    std::cout << "Test 2: CTE=0.5, TotalError=" << pid.TotalError() << std::endl;
    assert(std::abs(pid.TotalError() - 0.5) < 1e-6);

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
