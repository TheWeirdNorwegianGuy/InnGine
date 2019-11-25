#include "bspline.h"


BSpline::BSpline(std::vector<gsl::Vec3> controlPolygon, int degree)
{
    mControlPolygon = controlPolygon;

    mDegree = degree;

    mKnots = bufferKnots(generateKnots());
}

void BSpline::makeSpline(std::vector<gsl::Vec3> controlPolygon, int degree)
{
    mControlPolygon = controlPolygon;

    mDegree = degree;

    mKnots = bufferKnots(generateKnots());
}

gsl::Vec3 BSpline::findPoint(float t)
{
    gsl::Vec3 sumVertices;

    for(int i = 0; i <= mControlPolygon.size() - 1; i++)
    {
        if(validPoint(i, t, mDegree))
        {
            sumVertices = sumVertices + (mControlPolygon[i] * getWeight(i, mDegree, t));
        }
    }

    return sumVertices;
}

bool BSpline::validPoint(int i, float t, int k)
{
    return (t >= mKnots[i] && t < mKnots[i + k + 1]);
}

float BSpline::getWeight(int i, int j, float t)
{
    if(j > 0)
    {
        float J_Path = getWeight(i, j-1, t);
        float IJ_Path = getWeight(i+1, j-1, t);

        float J_Mult = 0;
        if((mKnots[i + j] - mKnots[i]) != 0)
        {
            J_Mult = ((t - mKnots[i])/(mKnots[i + j] - mKnots[i]));
        }

        float IJ_Mult = 0;
        if((mKnots[i + j + 1] - mKnots[i + 1]) != 0)
        {
            IJ_Mult = ((mKnots[i + j + 1] - t)/(mKnots[i + j + 1] - mKnots[i + 1]));
        }

        return ((J_Path * J_Mult) + (IJ_Path * IJ_Mult));
    }
    else if(validPoint(i, t))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

std::vector<float> BSpline::generateKnots()
{
    std::vector<float> returnVector;

    float nodeCount = (mControlPolygon.size() - mDegree) + 1;

    for(int i = 0; i < nodeCount; i++)
    {
        returnVector.push_back((float)i/(nodeCount - 1));
    }

    return returnVector;
}

std::vector<float> BSpline::bufferKnots(const std::vector<float> knots) const
{
    std::vector<float> newKnots;

    for(int i = 0; i < mDegree; i++)
    {
        newKnots.push_back(0);
    }

    for(int i = 0; i < knots.size(); i++)
    {
        newKnots.push_back(knots[i]);
    }

    for(int i = 0; i < mDegree; i++)
    {
        newKnots.push_back(knots[knots.size() - 1]);
    }

    return newKnots;
}



