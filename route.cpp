#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <map>
#include <set>
#include <limits>
#include <stack>
#include <cmath>

// (cat 4.list | ./normalize-list-specified bounds/4 | ./voronoi/voronoi -t; cat 4.geo | ./normalize-geo-specified bounds/4 ) | time ./route > foo.ps

struct point {
	double x;
	double y;
	std::map<size_t, double> neighbors;

	ssize_t came_from;
	double g_score;
	double f_score;
	bool necessary;

	point() {
	}

	point(double _x, double _y) {
		x = _x;
		y = _y;
	}
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

		dist = exp(log(dist) * 2);

		for (auto n = points[p1].neighbors.begin(); n != points[p1].neighbors.end(); ++n) {
			if (n->second == dist) {
				fprintf(stderr, "From %zu: same distance to %zu as to %zu\n", p1, n->first, p2);
			}
		}

		points[p1].neighbors.insert(std::pair<size_t, double>(p2, dist));
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

void bezier(std::vector<point> const &line) {
	if (line.size() < 2) {
		return;
	}

	printf("%.6f %.6f moveto ", line[0].x * 612, line[0].y * 612);

	for (size_t i = 0; i + 1 < line.size(); i++) {
		double x1 = line[i].x;
		double y1 = line[i].y;

		double x2 = line[i + 1].x;
		double y2 = line[i + 1].y;

		double x0 = x1, y0 = y1;
		if (i > 0) {
			x0 = line[i - 1].x;
			y0 = line[i - 1].y;
		}

		double x3 = x2, y3 = y2;
		if (i + 2 < line.size()) {
			x3 = line[i + 2].x;
			y3 = line[i + 2].y;
		}

		// http://www.antigrain.com/research/bezier_interpolation/

		// Assume we need to calculate the control
		// points between (x1,y1) and (x2,y2).
		// Then x0,y0 - the previous vertex,
		//      x3,y3 - the next one.

		double xc1 = (x0 + x1) / 2.0;
		double yc1 = (y0 + y1) / 2.0;
		double xc2 = (x1 + x2) / 2.0;
		double yc2 = (y1 + y2) / 2.0;
		double xc3 = (x2 + x3) / 2.0;
		double yc3 = (y2 + y3) / 2.0;

		double len1 = sqrt((x1-x0) * (x1-x0) + (y1-y0) * (y1-y0));
		double len2 = sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
		double len3 = sqrt((x3-x2) * (x3-x2) + (y3-y2) * (y3-y2));

		double k1 = len1 / (len1 + len2);
		double k2 = len2 / (len2 + len3);

		double xm1 = xc1 + (xc2 - xc1) * k1;
		double ym1 = yc1 + (yc2 - yc1) * k1;

		double xm2 = xc2 + (xc3 - xc2) * k2;
		double ym2 = yc2 + (yc3 - yc2) * k2;

		double smooth_value = .5;

		// Resulting control points. Here smooth_value is mentioned
		// above coefficient K whose value should be in range [0...1].
		double ctrl1_x = xm1 + (xc2 - xm1) * smooth_value + x1 - xm1;
		double ctrl1_y = ym1 + (yc2 - ym1) * smooth_value + y1 - ym1;

		double ctrl2_x = xm2 + (xc2 - xm2) * smooth_value + x2 - xm2;
		double ctrl2_y = ym2 + (yc2 - ym2) * smooth_value + y2 - ym2;

		printf("%.6f %.6f ", ctrl1_x * 612, ctrl1_y * 612);
		printf("%.6f %.6f ", ctrl2_x * 612, ctrl2_y * 612);
		printf("%.6f %.6f curveto ", x2 * 612, y2 * 612);
	}

	printf("%.6f %.6f lineto stroke\n", line[line.size() - 1].x * 612, line[line.size() - 1].y * 612);
}

static double square_distance_from_line(double point_x, double point_y, double segA_x, double segA_y, double segB_x, double segB_y) {
	double p2x = segB_x - segA_x;
	double p2y = segB_y - segA_y;
	double something = p2x * p2x + p2y * p2y;
	double u = 0 == something ? 0 : ((point_x - segA_x) * p2x + (point_y - segA_y) * p2y) / something;

	if (u > 1) {
		u = 1;
	} else if (u < 0) {
		u = 0;
	}

	double x = segA_x + u * p2x;
	double y = segA_y + u * p2y;

	double dx = x - point_x;
	double dy = y - point_y;

	return dx * dx + dy * dy;
}

// https://github.com/Project-OSRM/osrm-backend/blob/733d1384a40f/Algorithms/DouglasePeucker.cpp
static void douglas_peucker(std::vector<point> &geom, double e) {
	int start = 0;
	int n = geom.size();

	e = e * e;
	std::stack<int> recursion_stack;

	{
		int left_border = 0;
		int right_border = 1;
		// Sweep linerarily over array and identify those ranges that need to be checked
		do {
			if (geom[start + right_border].necessary) {
				recursion_stack.push(left_border);
				recursion_stack.push(right_border);
				left_border = right_border;
			}
			++right_border;
		} while (right_border < n);
	}

	while (!recursion_stack.empty()) {
		// pop next element
		int second = recursion_stack.top();
		recursion_stack.pop();
		int first = recursion_stack.top();
		recursion_stack.pop();

		double max_distance = -1;
		int farthest_element_index = second;

		// find index idx of element with max_distance
		int i;
		for (i = first + 1; i < second; i++) {
			double temp_dist = square_distance_from_line(geom[start + i].x, geom[start + i].y, geom[start + first].x, geom[start + first].y, geom[start + second].x, geom[start + second].y);

			double distance = std::fabs(temp_dist);

			if (distance > e && distance > max_distance) {
				farthest_element_index = i;
				max_distance = distance;
			}
		}

		if (max_distance > e) {
			// mark idx as necessary
			geom[start + farthest_element_index].necessary = 1;

			if (1 < farthest_element_index - first) {
				recursion_stack.push(first);
				recursion_stack.push(farthest_element_index);
			}
			if (1 < second - farthest_element_index) {
				recursion_stack.push(farthest_element_index);
				recursion_stack.push(second);
			}
		}
	}
}

static std::vector<point> simplify(std::vector<point> &line) {
	if (line.size() == 0) {
		return std::vector<point>();
	}
	for (size_t i = 0; i < line.size(); i++) {
		line[i].necessary = false;
	}
	line[0].necessary = 1;
	line[line.size() - 1].necessary = 1;

	douglas_peucker(line, .02); // .04 is about a mile

	std::vector<point> out;
	for (size_t i = 0; i < line.size(); i++) {
		if (line[i].necessary) {
			out.push_back(line[i]);
		}
	}
	return out;
}

point jitter(std::vector<point> const &points, size_t i) {
	double xd = 0, yd = 0, sum = 0;
	for (auto n = points[i].neighbors.begin(); n != points[i].neighbors.end(); ++n) {
		double weight = rand() % 1000;
		xd += (points[n->first].x - points[i].x) * weight;
		yd += (points[n->first].y - points[i].y) * weight;
		sum += weight;
	}
	xd /= sum;
	yd /= sum;
	return point(points[i].x + xd, points[i].y + yd);
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

#if 0
	// Normalize distances to shortest

	double min = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < points.size(); i++) {
		for (auto j = points[i].neighbors.begin(); j != points[i].neighbors.end(); ++j) {
			if (j->second < min) {
				min = j->second;
			}
		}
	}
	for (size_t i = 0; i < points.size(); i++) {
		std::map<size_t, double> out;
		for (auto j = points[i].neighbors.begin(); j != points[i].neighbors.end(); ++j) {
			out.insert(std::pair<size_t, double>(j->first, j->second / min));
		}
		points[i].neighbors = out;
	}
#endif

	while (fgets(s, 2000, stdin)) {
		double x1, y1, x2, y2;

		if (sscanf(s, "%lf %lf %lf %lf", &x1, &y1, &x2, &y2) != 4) {
			fprintf(stderr, "???2 %s", s);
			continue;
		}

		size_t n1 = find_node(points, x1, y1);
		size_t n2 = find_node(points, x2, y2);

		std::vector<size_t> route = astar(points, n1, n2);

		std::vector<point> line;
		line.push_back(point(x2, y2));
		for (size_t i = 0; i < route.size(); i++) {
			line.push_back(jitter(points, route[i]));
			// line.push_back(points[route[i]]);
		}
		line.push_back(point(x1, y1));

		line = simplify(line);
		bezier(line);

#if 0
		for (size_t i = 0; i < line.size(); i++) {
			printf("%.6f %.6f %s ", line[i].x * 612, line[i].y * 612, i == 0 ? "moveto" : "lineto");
		}
		printf("stroke\n");
#endif

		fflush(stdout);
	}
}
