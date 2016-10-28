#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <map>
#include <set>
#include <limits>

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

	if (points[p].x != 0 && (points[p].x != x || points[p].y != y)) {
		fprintf(stderr, "UGH %zu\n", p);
	}

	points[p].x = x;
	points[p].y = y;
}

void addneighbor(std::vector<point> &points, size_t p1, size_t p2) {
	if (points[p1].neighbors.count(p2) == 0) {
		double xd = points[p1].x - points[p2].x;
		double yd = points[p1].y - points[p2].y;
		double dist = sqrt(xd * xd + yd * yd);

		points[p1].neighbors.insert(std::pair<size_t, double>(p2, exp(log(dist) * 2)));
	}
}

size_t find_node(std::vector<point> const &points, double x, double y) {
	size_t best = 0;
	double bestdsq = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i].neighbors.size() > 0) {
			double xd = points[i].x - x;
			double yd = points[i].y - y;
			double dsq = xd * xd + yd * yd;

			if (dsq < bestdsq) {
				//printf("%f,%f: %f,%f %f is better than %f\n", x, y, points[i].x, points[i].y, dsq, bestdsq);
				best = i;
				bestdsq = dsq;
			}

		}
	}
	return best;
}

double heur(std::vector<point> const &points, size_t n1, size_t n2) {
	return 0;

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

std::vector<size_t> astar(std::vector<point> &points, size_t start, size_t goal) {
	std::set<size_t> closed_set;

	std::set<size_t> open_set;
	open_set.insert(start);

	for (size_t i = 0; i < points.size(); i++) {
		points[i].g_score = std::numeric_limits<double>::infinity();
		points[i].f_score = std::numeric_limits<double>::infinity();
		points[i].came_from = -1;
	}

	points[start].g_score = 0;
	points[start].f_score = heur(points, start, goal);

	while (open_set.size() != 0) {
		size_t current;

#if 1
		bool first = true;
		for (auto i = open_set.begin(); i != open_set.end(); ++i) {
			if (first) {
				//printf("defaulting to %zu, f_score %f\n", *i, points[*i].f_score);
				current = *i;
				first = false;
			} else if (points[*i].f_score < points[current].f_score) {
				//printf("better: %zu, f_score %f\n", *i, points[*i].f_score);
				current = *i;
			}
		}
#else
		for (auto i = open_set.begin(); i != open_set.end(); ++i) {
			if (i == open_set.begin()) {
				current = *i;
			} else if (points[*i].g_score < points[current].g_score) {
				current = *i;
			}
		}
#endif

		//printf("so current is %zu, g_score %f\n", current, points[current].g_score);

		if (current == goal) {
			return reconstruct(points, current);
		}

		open_set.erase(open_set.find(current));
		closed_set.insert(current);

		for (auto n = points[current].neighbors.begin(); n != points[current].neighbors.end(); ++n) {
			size_t neighbor = n->first;

			if (closed_set.count(neighbor) > 0) {
				//printf("%zu is closed\n", neighbor);
				continue;
			}

			double tentative_g_score = points[current].g_score + n->second;
			//printf("%zu tentatively costs %f\n", neighbor, tentative_g_score);

			if (open_set.count(neighbor) == 0) {
				open_set.insert(neighbor);
			} else if (tentative_g_score < points[neighbor].g_score) {
				continue;
			}

			points[neighbor].came_from = current;
			points[neighbor].g_score = tentative_g_score;
			points[neighbor].f_score = tentative_g_score + heur(points, neighbor, goal);
		}
	}

	return std::vector<size_t>();
}

int main(int argc, char **argv) {
	char s[2000];

	std::vector<point> points;

	printf("0 setlinewidth\n");
	printf(".1 .setopacityalpha\n");

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

		printf("%.6f %.6f moveto ", x2 * 612, y2 * 612);
		for (size_t i = 0; i < route.size(); i++) {
			printf("%.6f %.6f %s ", points[route[i]].x * 612, points[route[i]].y * 612, i == 0 ? "lineto" : "lineto");
		}
		printf("%.6f %.6f lineto ", x1 * 612, y1 * 612);
		printf("stroke\n");
		fflush(stdout);
	}
}
