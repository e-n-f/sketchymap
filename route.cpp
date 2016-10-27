#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <map>

struct point {
	double x;
	double y;
	std::map<size_t, double> neighbors;
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

	points[p1].neighbors.insert(std::pair<size_t, double>(p2, sqrt(xd * xd + yd * yd)));
}

int main(int argc, char **argv) {
	char s[2000];

	std::vector<point> points;

	while (fgets(s, 2000, stdin)) {
		int p1, p2, p3;
		double x1, y1, x2, y2, x3, y3;

		if (strcmp(s, "end\n") == 0) {
			break;
		}

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

	// Normalize distances to longest

	double max = 0;
	for (size_t i = 0; i < points.size(); i++) {
		for (auto j = points[i].neighbors.begin(); j != points[i].neighbors.end(); ++j) {
			if (j->second > max) {
				max = j->second;
			}
		}
	}
	for (size_t i = 0; i < points.size(); i++) {
		std::map<size_t, double> out;
		for (auto j = points[i].neighbors.begin(); j != points[i].neighbors.end(); ++j) {
			out.insert(std::pair<size_t, double>(j->first, j->second / max));
		}
		points[i].neighbors = out;
	}
}
