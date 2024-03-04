#ifndef NXLIBTRANSFORMATIONHELPER_H
#define NXLIBTRANSFORMATIONHELPER_H
#include <QObject>

class NxLibTransformationHelper
{
public:
    static void TransformationJSONToEulerAngles(std::string json, double &x, double &y, double &z, double &a, double &b, double &c);
    static std::string EulerAnglesToTransformationJSON(double x, double y, double z, double a, double b, double c);
    static std::string AngleAxisToTransformationJSON(double x, double y, double z, double rx, double ry, double rz, double alpha);
    static std::string ChainTransformationsJSON(std::string tfLast, std::string tfFirst);
};

#endif // NXLIBTRANSFORMATIONHELPER_H
