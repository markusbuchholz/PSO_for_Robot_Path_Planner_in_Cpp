// Markus Buchholz
// g++ pso_robot.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <algorithm>
#include <random>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//---------------------------------------------------------------------------------------------

int EVOLUTIONS = 3000;
int PARTICLES = 200;
float C1 = 1.5;
float C2 = 1.5;
float W = 0.9; // inertia weight
float x1min = 0.0;
float x1max = 50.0;
float x2min = 0.0;
float x2max = 50.0;

float alpha1 = 1.9;
float alpha2 = 1.7;
float alpha3 = 0.25;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//---------------------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};

//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::uniform_real_distribution<float> distrib(0.0, 1.0);
    return distrib(engine);
}

//--------------------------------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < PARTICLES; ii++)
    {

        pos.push_back({valueGenerator(x1min, x1max), valueGenerator(x2min, x2max)});
    }

    return pos;
}

//--------------------------------------------------------------------------------

std::vector<Pos> initVelocityXY()
{

    std::vector<Pos> velo;

    for (int ii = 0; ii < PARTICLES; ii++)
    {

        velo.push_back({valueGenerator(0, 1), valueGenerator(0, 1)});
    }

    return velo;
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {

        float drObs = std::sqrt(std::pow(obsX - ii.x, 2) + std::pow(obsY - ii.y, 2));
        float drGoal = std::sqrt(std::pow(goalX - ii.x, 2) + std::pow(goalY - ii.y, 2));
        float angle = std::atan2((goalY - ii.y), (goalX - ii.x)) * 180 / M_PI;

        float cost = alpha1 * drGoal + alpha2 / drObs + obsR * alpha3 * angle;

        funcValue.push_back(cost);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}
//--------------------------------------------------------------------------------

bool compareMax(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second > b.second;
}

//-------------------------------------------------------------------------------

std::pair<Pos, float> findBestValue(std::vector<Pos> pos, std::vector<float> func)
{
    std::vector<std::pair<Pos, float>> temp;

    for (int ii = 0; ii < func.size(); ii++)
    {

        temp.push_back(std::pair<Pos, float>(pos[ii], func[ii]));
    }

    std::sort(temp.begin(), temp.end(), compareMax);

    return temp[0];
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    float drObs = std::sqrt(std::pow(obsX - pos.x, 2) + std::pow(obsY - pos.y, 2));
    float drGoal = std::sqrt(std::pow(goalX - pos.x, 2) + std::pow(goalY - pos.y, 2));
    float angle = std::atan2((goalY - pos.y), (goalX - pos.x));

    float cost = alpha1 * drGoal + alpha2 / drObs + obsR * alpha3 * angle;

    return cost;
}

//--------------------------------------------------------------------------------

Pos velocityUpdate(Pos gBest, Pos pBest, Pos actPos, Pos actVelo)
{

    float vnew_x = W * actVelo.x + C1 * generateRandom() * (pBest.x - actPos.x) + C2 * generateRandom() * (gBest.x - actPos.x);
    float vnew_y = W * actVelo.y + C1 * generateRandom() * (pBest.y - actPos.y) + C2 * generateRandom() * (gBest.y - actPos.y);

    return {vnew_x, vnew_y};
}
//--------------------------------------------------------------------------------

Pos positionUpdate(Pos actualPos, Pos vnew)
{

    Pos Pnew;

    Pnew.x = actualPos.x + vnew.x;
    Pnew.y = actualPos.y + vnew.y;

    if (Pnew.x < x1min)
    {
        Pnew.x = x1min;
    }

    if (Pnew.x > x1max)
    {
        Pnew.x = x1max;
    }

    if (Pnew.y < x2min)
    {
        Pnew.y = x2min;
    }

    if (Pnew.y > x2max)
    {
        Pnew.y = x2max;
    }

    return Pnew;
}

//--------------------------------------------------------------------------------
std::vector<Pos> runPSO()
{

    std::vector<Pos> initVelocities = initVelocityXY(); // (0-1)
    std::vector<Pos> initPositions = initPosXY();
    std::vector<float> funcValue = function(initPositions);

    std::vector<Pos> pBestPositions = initPositions;

    std::vector<Pos> pPositions = initPositions;
    std::vector<Pos> pVelocities = initVelocities;

    std::vector<Pos> pPositionsNew = pPositions;
    std::vector<Pos> pVelocitiesNew = pVelocities;
    // std::vector<float> actualfuncValue = funcValue;

    std::pair<Pos, float> gBest = findBestValue(pPositions, funcValue);

    Pos gBestPos = gBest.first;

    float gBestValue = gBest.second;

    for (int jj = 0; jj < EVOLUTIONS; jj++)
    {

        for (int ii = 0; ii < PARTICLES; ii++)
        {

            pVelocitiesNew[ii] = velocityUpdate(gBestPos, pBestPositions[ii], pPositions[ii], pVelocities[ii]);
            pPositionsNew[ii] = positionUpdate(pVelocitiesNew[ii], pPositions[ii]);
            float pfunc = func(pPositionsNew[ii]);

            if (pfunc > gBestValue)
            {
                gBestValue = pfunc;
                gBestPos = pPositionsNew[ii];
            }
            if (pfunc > funcValue[ii])
            {
                pBestPositions[ii] = pPositionsNew[ii];
                funcValue[ii] = pfunc;
            }
       
        }

            pPositions = pPositionsNew;
            pVelocities = pVelocitiesNew;

    }
    return pBestPositions;
}

//--------------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> gen_circle(float a, float b, float r)
{

    std::vector<float> xX;
    std::vector<float> yY;

    for (float dt = -M_PI; dt < M_PI; dt += 0.01)
    {

        xX.push_back(a + r * std::cos(dt));
        yY.push_back(b + r * std::sin(dt));
    }
    return std::make_tuple(xX, yY);
}

//----------------------------------------------------------------------------------------------

void plot2D(std::vector<float> xX, std::vector<float> yY)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());

    std::tuple<std::vector<float>, std::vector<float>> circle = gen_circle(obsX, obsY, obsR);

    std::vector<float> xObs = std::get<0>(circle);
    std::vector<float> yObs = std::get<1>(circle);

    plt::plot(xX, yY);
    plt::plot(xObs, yObs);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}

//---------------------------------------------------------------------------------------------
int main()
{
    std::vector<Pos> path = runPSO();

    std::vector<float> xX;
    std::vector<float> yY;

    for (auto &ii : path)
    {
        xX.push_back(ii.x);
        yY.push_back(ii.y);
    }

    plot2D(xX, yY);
}
