#ifndef __UI_QT_COLORMAPS_H__
#define __UI_QT_COLORMAPS_H__

#include <QtGui>
#include <string>

QVector<QRgb> colormap(std::string const& name, int numElements = 256);

#endif /* __UI_QT_COLORMAPS_H__ */
