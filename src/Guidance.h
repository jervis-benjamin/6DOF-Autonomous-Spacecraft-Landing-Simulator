#pragma once

#include <Eigen/Dense>
#include "Dynamics.h"
#include "SpacecraftConfig.h"
#include "QuaternionTools.h"
#include "World.h"
using namespace std;

class Guidance{

private:
    const Dynamics& dynamics;
    const World& world;

public:
    Guidance(const Dynamics& dyn, const World& w): dynamics(dyn), world(w){}

};