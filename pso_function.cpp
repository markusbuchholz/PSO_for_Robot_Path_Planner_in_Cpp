//Markus Buchholz

#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <algorithm>
#include <random>

//---------------------------------------------------------------------------------------------

int EVOLUTIONS = 500;
int PARTICLES = 50;
float C1 = 1.5;
float C2 = 1.5;
float W = 0.9; // inertia weight
float x1min = -5.0;
float x1max = 5.0;
float x2min = -5.0;
float x2max = 5.0;

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

        // funcValue.push_back(ii.x * ii.x - ii.x * ii.y + ii.y * ii.y + 2 * ii.x + 4 * ii.y + 3);
        funcValue.push_back(ii.x * ii.y);
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

    std::sort(temp.begin(), temp.end(), compareMin);

    return temp[0];
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    // return (pos.x * pos.x - pos.x * pos.y + pos.y * pos.y + 2 * pos.x + 4 * pos.y + 3);
    return (pos.x * pos.y);
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
void runPSO()
{

    std::vector<Pos> initVelocities = initVelocityXY(); // (0-1)
    std::vector<Pos> initPositions = initPosXY();
    std::vector<float> funcValue = function(initPositions);

    std::vector<Pos> pBest = initPositions;
    std::vector<Pos> pActual = initPositions;
    std::vector<Pos> pVelocities = initVelocities;
    std::vector<float> actualfuncValue = funcValue;

    for (int jj = 0; jj < EVOLUTIONS; jj++)
    {
        std::pair<Pos, float> gBest = findBestValue(initPositions, actualfuncValue);

        float gBestValue = gBest.second;

        for (int ii = 0; ii < PARTICLES; ii++)
        {

            Pos veloNew = velocityUpdate(gBest.first, pBest[ii], pActual[ii], pVelocities[ii]);
            Pos posNew = positionUpdate(veloNew, pActual[ii]);
            float pfunc = func(posNew);

            if (pfunc < gBestValue)
            {

                gBestValue = pfunc;
            }
            if (pfunc < actualfuncValue[ii])
            {
                pBest[ii] = posNew;
                actualfuncValue[ii] = pfunc;
            }

            for (int ii = 0; ii < pBest.size(); ii++)
            {

                std::cout << "x : " << pBest[ii].x << " y :" << pBest[ii].y << "\n";
            }

            std::cout << "global best " << gBestValue << "\n";
        }
    }
}

//---------------------------------------------------------------------------------------------
int main()
{
    runPSO();
}