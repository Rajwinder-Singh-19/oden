/*
SPRING DAMPER CONTACT SYSTEM

THIS IS A VERY SMALL DEMONSTRATION OF WHAT ODEN CAN DO
HERE, AN OBJECT FALLING FROM A HEIGHT ONTO GROUND IS BEING SIMULATED,
USING A SPRING DAMPER DIFFERENTIAL EQUATION MODEL, 
A VERY BASIC MODEL IN THE DOMAIN OF CONTACT MECHANICS.

THE MODEL IS INTEGRATED USING ODEN'S ADAPTIVE STEP SIZE EULER SOLVER.
AS MORE ADVANCED SCHEMES ARE INTEGRATED INTO ODEN, STIFFER ODE SIMULATIONS CAN BE CONDUCTED

RESULTS ARE SAVED IN BUILD/GCC/DEBUG/OUTPUT.CSV
*/

#include <iostream>
#include <oden.h>
#include <fstream>

struct Overlap
{
    bool hasOverlap;
    double overlapDepth;
};

struct ContactStatus
{
    bool hasOverlap;
    double overlapDepth;
};


// CONTACT IS AT x = 0
ContactStatus overlapDepth(double objectPosition)
{
    ContactStatus overlapStatus{false, 0.0};

    if (objectPosition < 0)
    {
        overlapStatus.hasOverlap = true;
        overlapStatus.overlapDepth = std::fabs(objectPosition);
    }

    return overlapStatus;
}

int main()
{
    // ALL PHYSICAL QUANTITIES IN SI UNITS
    const double CONTACT_STIFFNESS_FACTOR = 100000; // High stiffness given the fact ground is inelastic
    const double CONTACT_DAMPING_FACTOR = 200; // Damping factor ensures object does not bounce back to original height
    const double ACCELERATION_GRAVITY = -9.81; 
    const double OBJECT_MASS = 10; 

    std::vector X{10.0, 0.0}; // Initial Conditions, Intitial Position is 10, and Initial Velocity is 0

    oden::System contactSystem(X);

    auto contactStateFunction = [&](double t, const decltype(contactSystem) &system) {
        auto objectState = system[0];

        auto objectPosition = objectState[0];
        auto objectVelocity = objectState[1];

        auto objectOverlapStatus = overlapDepth(objectPosition);

        double contactAcceleration = CONTACT_STIFFNESS_FACTOR * objectOverlapStatus.overlapDepth / OBJECT_MASS;

        double contactDampingAccleration = 0;

        if (objectOverlapStatus.hasOverlap)
        {
            contactDampingAccleration = -CONTACT_DAMPING_FACTOR * objectVelocity / OBJECT_MASS;
        }

        double objectGravityAcceleration = ACCELERATION_GRAVITY;
        double totalObjectAcceleration = contactAcceleration + contactDampingAccleration + objectGravityAcceleration;

        return std::vector{totalObjectAcceleration};
    };

    contactSystem.bindStateFunction(contactStateFunction);

    oden::ControlParameters simulationControl{0, 25, 0.01, 0.0001, 1e-6, 1e-4};
    auto result = contactSystem.EulerSolver(simulationControl, true);

    // Saving the data in a file
    std::ofstream file("output.csv");

    file << "Time (s)," << "Object Position (m)\n";

    for (auto &&sol : result)
    {
        file << sol.time << ",";

        for (auto &&i : sol.variables)
        {
            file << i << ",\n";
        }
        
    }

    return 0;
}