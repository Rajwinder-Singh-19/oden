# oden: Modern C++ Template Library For N-Dimensional ODE Systems
This repository is currently in development. There is a full-fledged working explicit Euler solver implementaton available in the [oden.h](/include/oden.h) header file.
Just download the header file and copy into your project include folder if you wish to use the implementation. The user API is quite easy to use, just go through [main.cpp](/src/main.cpp), which models a simple harmonic oscillator ODE using oden. Make sure you have the latest GCC compiler installed on your system to build with oden. Building with Clang is currently untested. Thanks...

# What can oden do right now
It might surprise you, but even with an euler solver, you can simulate very interesting phenomena. Take for example, contact mechanics which has huge applications in robotics.
oden can easily model mass-damper style contact systems. Below is an example of a mass damper system - An imperfectly elastic object falling from some height on to ground, under the influence of gravity.

<img width="1920" height="959" alt="Oden Simulation" src="https://github.com/user-attachments/assets/3e521540-a63d-495b-9c67-32898243fc5b" />
