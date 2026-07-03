# oden: Modern C++ Template Library For N-Dimensional ODE Systems
This repository is currently in development. There is a full-fledged working explicit Euler solver implementaton available in the [oden.h](/include/oden.h) header file.
Just download the header file and copy into your project include folder if you wish to use the implementation. The user API is quite easy to use, just go through [main.cpp](/src/main.cpp), which models a simple harmonic oscillator ODE using oden. For a more advanced example, go through [spring_damper_contact.cpp](/src/spring_damper_contact.cpp), which models the contact mechanics. Make sure you have the latest GCC compiler installed on your system to build with oden. Building with Clang is currently untested. Thanks...

# What can oden do
Even with a basic Euler scheme, very interesting phenomena can be simulated. Take for example, contact mechanics, which has many applications in robotics.
oden can easily model spring damper contact systems. Below is an example - An elastic object falling from some height on to ground, under the influence of gravity (see [spring_damper_contact.cpp](/src/spring_damper_contact.cpp)).

<img width="1920" height="959" alt="Oden Simulation" src="https://github.com/user-attachments/assets/3361c77c-2dc3-4362-97fa-ce2756e5229c" />

