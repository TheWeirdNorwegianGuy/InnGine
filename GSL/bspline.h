#ifndef BSPLINE_H
#define BSPLINE_H

#include "vec3.h"
#include <vector>


class BSpline
{
private:

    std::vector<gsl::Vec3> mControlPolygon;

    int mDegree;

    std::vector<float> mKnots;

public:

    BSpline(std::vector<gsl::Vec3> controlPolygon, int degree);

    void makeSpline(std::vector<gsl::Vec3> controlPolygon, int degree);

    gsl::Vec3 findPoint(float t);

    bool validPoint(int i, float t, int k = 0);

    float getWeight(int i, int j, float t);

    std::vector<float> generateKnots();
    std::vector<float> bufferKnots(const std::vector<float> knots) const;


};

#endif // BSPLINE_H
