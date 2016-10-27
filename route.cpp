#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

struct point {
	double x;
	double y;
	std::vector<size_t> neighbors;
	std::vector<double> distances;
};

void addpoint(std::vector<point> &points, size_t p, double x, double y) {
	if (p >= points.size()) {
		points.resize(p + 1);
	}

	points[p].x = x;
	points[p].y = y;
}

void addneighbor(std::vector<point> &points, size_t p1, size_t p2) {
	double xd = points[p1].x - points[p2].x;
	double yd = points[p1].y - points[p2].y;

	points[p1].neighbors.push_back(p2);
	points[p2].distances.push_back(sqrt(xd * xd + yd * yd));
}

int main(int argc, char **argv) {
	char s[2000];

	std::vector<point> points;

	while (fgets(s, 2000, stdin)) {
		int p1, p2, p3;
		double x1, y1, x2, y2, x3, y3;

		if (sscanf(s, "%d %lf,%lf %d %lf,%lf %d %lf,%lf",
			&p1, &x1, &y1, &p2, &x2, &y2, &p3, &x3, &y3) != 9) {
			fprintf(stderr, "??? %s", s);
			continue;
		}

		addpoint(points, p1, x1, y1);
		addpoint(points, p2, x2, y2);
		addpoint(points, p3, x3, y3);

		addneighbor(points, p1, p2);
		addneighbor(points, p1, p3);
		addneighbor(points, p2, p1);
		addneighbor(points, p2, p3);
		addneighbor(points, p3, p1);
		addneighbor(points, p3, p2);
	}
}
