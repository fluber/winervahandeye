#include "qtColormaps.h"
#include <algorithm>

template <typename T>
inline T interpolateLinearly(T x1, T y1, T x2, T y2, T x) {
	return y1 + (x - x1) * (y2-y1) / (x2-x1); // Geht das vielleicht geschickter?
};

QVector<QRgb> colormap(std::string const& name, int numElements) {

	QVector<QRgb> map;

	std::string mapNameLower; mapNameLower.resize(name.size());
	std::transform(name.begin(), name.end(), mapNameLower.begin(), tolower);

	if (mapNameLower.compare("jet") == 0) {
		// Generate Matlab "Jet" colormap
		for (int i = 0; i < numElements; i++) {
			double p0 = -0.5/4;
			double p1 =  0.5/4;
			double p2 =  1.5/4;
			double p3 =  2.5/4;
			double p4 =  3.5/4;
			double p5 =  4.5/4;

			double p = (double)i / (numElements-1);

			double r, g, b;
			if (p < p1) {
				r = 0;
				g = 0;
				b = interpolateLinearly<double>(p0, 0, p1, 255, p);
			} else if (p < p2) {
				r = 0;
				g = interpolateLinearly<double>(p1, 0, p2, 255, p);
				b = 255;
			} else if (p < p3) {
				r = interpolateLinearly<double>(p2, 0, p3, 255, p);
				g = 255;
				b = interpolateLinearly<double>(p2, 255, p3, 0, p);
			} else if (p < p4) {
				r = 255;
				g = interpolateLinearly<double>(p3, 255, p4, 0, p);
				b = 0;
			} else {
				r = interpolateLinearly<double>(p4, 255, p5, 0, p);
				g = 0;
				b = 0;
			}
			QRgb color = qRgb((int)(r+0.5), (int)(g+0.5), (int)(b+0.5));
			map.append(color);
		}
	} else if (mapNameLower.compare("gray") == 0) {
		// Generate gray scale map
		for (int i = 0; i < numElements; i++) {
			QRgb color = qRgb(i, i, i);
			map.append(color);
		}
	}
	return map;
}
