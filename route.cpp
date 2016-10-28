#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <map>
#include <set>

struct point {
	double x;
	double y;
	std::map<size_t, double> neighbors;

	ssize_t came_from;
	double g_score;
	double f_score;
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

size_t find_node(std::vector<point> const &points, double x, double y) {
	size_t best = 0;
	double bestdsq = 999;
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i].neighbors.size() > 0) {
			double xd = points[i].x - x;
			double yd = points[i].x - y;
			double dsq = xd * xd + yd * yd;

			if (dsq < bestdsq) {
				best = i;
				bestdsq = dsq;
			}
		}
	}
	return best;
}

double heur(std::vector<point> const &points, size_t n1, size_t n2) {
	double xd = points[n1].x - points[n2].x;
	double yd = points[n1].y - points[n2].y;
	return sqrt(xd * xd + yd * yd);
}

std::vector<size_t> reconstruct(std::vector<point> const &points, size_t current) {
	std::vector<size_t> path;

	path.push_back(current);
	while (points[current].came_from >= 0) {
		current = points[current].came_from;
		path.push_back(current);
	}

	return path;
}

std::vector<size_t> astar(std::vector<point> &points, size_t n1, size_t n2) {
	std::set<size_t> closed_set;

	std::set<size_t> open_set;
	open_set.insert(n1);

	for (size_t i = 0; i < points.size(); i++) {
		points[i].g_score = 999;
		points[i].f_score = 999;
		points[i].came_from = -1;
	}

	points[n1].g_score = 0;
	points[n1].f_score = heur(points, n1, n2);

	while (open_set.size() != 0) {
		size_t current ;

		for (auto i = open_set.begin(); i != open_set.end(); ++i) {
			if (i == open_set.begin()) {
				current = *i;
			} else if (points[*i].f_score < points[current].f_score) {
				current = *i;
			}
		}

		if (current == n2) {
			return reconstruct(points, current);
		}

		open_set.erase(open_set.find(current));
		closed_set.insert(current);

		for (auto n = points[current].neighbors.begin(); n != points[current].neighbors.end(); ++n) {
			if (closed_set.count(n->first) > 0) {
				continue;
			}

			double tentative_g_score = points[current].g_score + n->second;

			if (open_set.count(n->first) == 0) {
				open_set.insert(n->first);
			} else if (tentative_g_score < points[n->first].g_score) {
				continue;
			}

			points[n->first].came_from = current;
			points[n->first].g_score = tentative_g_score;
			points[n->first].f_score = tentative_g_score + heur(points, n->first, n2);
		}
	}

	return std::vector<size_t>();
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

	while (fgets(s, 2000, stdin)) {
		double x1, y1, x2, y2;

		if (sscanf(s, "%lf %lf %lf %lf", &x1, &y1, &x2, &y2) != 4) {
			fprintf(stderr, "???2 %s", s);
			continue;
		}

		size_t n1 = find_node(points, x1, y1);
		size_t n2 = find_node(points, x2, y2);

		std::vector<size_t> route = astar(points, n1, n2);

		for (size_t i = 0; i < route.size(); i++) {
			printf("%.6f %.6f %s ", points[route[i]].x * 612, points[route[i]].y * 612, i == 0 ? "moveto" : "lineto");
		}
		printf("stroke\n");
	}
}
