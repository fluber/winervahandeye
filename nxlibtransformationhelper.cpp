#define _USE_MATH_DEFINES
#include "nxlibtransformationhelper.h"
#include "nxLib.h"
#include "math.h"

void NxLibTransformationHelper::TransformationJSONToEulerAngles(std::string json, double &x, double &y, double &z, double &a, double &b, double &c)
{
    NxLibCommand convert(cmdConvertTransformation);
    convert.parameters()[itmTransformation].setJson(json, false);
    convert.parameters()[itmSplitRotation].set(valXYZ);

    convert.execute();

    NxLibItem tf = convert.result()[itmTransformations];
    x = tf[0][itmTranslation][0].asDouble();
    y = tf[0][itmTranslation][1].asDouble();
    z = tf[0][itmTranslation][2].asDouble();
    c = tf[0][itmRotation][itmAngle].asDouble();
    b = tf[1][itmRotation][itmAngle].asDouble();
    a = tf[2][itmRotation][itmAngle].asDouble();
}

std::string NxLibTransformationHelper::EulerAnglesToTransformationJSON(double x, double y, double z, double a, double b, double c)
{
    NxLibCommand chain(cmdChainTransformations);
    NxLibItem tf = chain.parameters()[itmTransformations];

    tf[0].setJson(AngleAxisToTransformationJSON(x, y, z, 0, 0, 1, c * M_PI / 180), false);
    tf[1].setJson(AngleAxisToTransformationJSON(0, 0, 0, 0, 1, 0, b * M_PI / 180), false);
    tf[2].setJson(AngleAxisToTransformationJSON(0, 0, 0, 1, 0, 0, a * M_PI / 180), false);

    chain.execute();

    return chain.result()[itmTransformation].asJson(false, 15, true);
}

std::string NxLibTransformationHelper::AngleAxisToTransformationJSON(double x, double y, double z, double rx, double ry, double rz, double alpha)
{
    NxLibItem tf("/tmpTF");
    tf[itmTranslation][0].set(x);
    tf[itmTranslation][1].set(y);
    tf[itmTranslation][2].set(z);

    tf[itmRotation][itmAngle].set(alpha);
    tf[itmRotation][itmAxis][0].set(rx);
    tf[itmRotation][itmAxis][1].set(ry);
    tf[itmRotation][itmAxis][2].set(rz);

    std::string result = tf.asJson(false, 15, true);
    tf.erase();
    return result;
}

std::string NxLibTransformationHelper::ChainTransformationsJSON(std::string tfLast, std::string tfFirst)
{
    NxLibCommand chain(cmdChainTransformations);
    NxLibItem tf = chain.parameters()[itmTransformations];

    tf[0].setJson(tfLast, false);
    tf[1].setJson(tfFirst, false);

    chain.execute();

    return chain.result()[itmTransformation].asJson(false, 15, true);

}
