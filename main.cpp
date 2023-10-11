////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

using namespace std;

//defining Point and Polygon typedefs
typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

// compute determinant
double inline det(const Point &u, const Point &v) {
	return u.real() * v.imag() - u.imag() * v.real();
}

//check if point q lies between points p and point r
bool onRay(const Point &p, const Point &q, const Point &r) {
    // Check if q lies between p and r (both inclusive)
    return q.real() <= std::max(p.real(), r.real()) && 
           q.real() >= std::min(p.real(), r.real()) &&
           q.imag() <= std::max(p.imag(), r.imag()) &&
           q.imag() >= std::min(p.imag(), r.imag());
}


// Return true iff [a,b] intersects [c,d], and store the intersection in ans. return bool.
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &intersection) {
    Point vectorAB = b - a;
    Point vectorCD = d - c;

    double crossProduct = det(vectorAB, vectorCD);
    double collinearityCheck = det(c - a, vectorAB);

    if (crossProduct == 0) {
        // Collinear
        if (collinearityCheck == 0) {
            // Check if segment CD overlaps with segment AB
            if (onRay(a, c, b) || onRay(a, d, b) || onRay(c, a, d) || onRay(c, b, d)) {
				
                return true;
            }
        }

        // Collinear but disjoint

        return false;
    }
    
	// t and u are scalar parameters for linear interpolation involving vectors AB and CD. we use these to figure out the intersection point between the two segments
	double t = det(c - a, vectorCD) / crossProduct;
    double u = collinearityCheck / crossProduct;

    if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
        intersection = a + t * vectorAB;
        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {

	//define vector containing points that are on ray, so we do not visit these points again. 
	std::vector<Point> onRay;
	   
	// minX, minY => Largest possible initially; maxX, maxY => Smallest possible initially
    double minX = std::numeric_limits<double>::max(), maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max(), maxY = std::numeric_limits<double>::lowest();

	//iterate through points to find bounding max and min X and Y coordinates of the polygon.
    for (const Point &p : poly) {
        minX = std::min(minX, p.real());
        maxX = std::max(maxX, p.real());
        minY = std::min(minY, p.imag());
        maxY = std::max(maxY, p.imag());
    }

	//point guaranteed to be outside the polygon. We can make it easier upon ourselves and have an irrational value here, to avoid colinear lines. We do not do so in this exercise however.
    Point outside(maxX + 1, maxY + 1);

	// cout << "point: (" << query.real() << ", " << query.imag() <<  "). outside: (" << maxX+1 << ", " << maxY+1 <<")" <<endl;
    
	int intersections = 0;

    for (int i = 0; i < poly.size(); i++) {

		
        Point p1 = poly[i];
        Point p2 = poly[(i + 1) % poly.size()];  // To ensure the loop closes
		// cout <<"p1 p2: " << p1 << p2 <<endl;

		//if any of the points that we've iterated to is on the onRay list, it means that we've come here again. we skip these.
		if((std::find(onRay.begin(), onRay.end(), p1) != onRay.end()) || (std::find(onRay.begin(), onRay.end(), p2) != onRay.end()) ) {
			continue;
		}

		//dummy will hold the intersection point, if the line segments intersect.
        Point dummy;
        if (intersect_segment(query, outside, p1, p2, dummy)) {


			// if intersection point is the same as a point on the polygon's edge, it means that we're visiting the edge for the first and only time (we checked above if we encountered it before.)
			if(p1 == dummy) {
				// cout <<"segment intersects with vertex p1 " << p1 <<endl;
				onRay.push_back(dummy);
			}
			else if (p2 == dummy) {
				// cout <<"segment intersects with vertex p2 " << p2 <<endl;
				onRay.push_back(dummy);
			}
			// cout << "++intersection: poly points: " << p1 << ", " << p2 << ". DUMMY=> " << dummy.real() << ", " << dummy.imag() << endl;
            ++intersections;
        }
    }
	// cout << "intersections: " << intersections <<endl;
    return intersections % 2 == 1;  // Odd number of intersections implies inside.
}

////////////////////////////////////////////////////////////////////////////////
// FILE MANIPULATION:

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream file(filename);


	if (!file.is_open()) {
		// TODO: throw exception
		throw std::runtime_error(" Error! Failed to open file " + filename);
	}
	int n;
	file >> n;
	double x, y, z; 
	for (int i = 0; i < n; ++i) {

		// read x, y, z from file
		file >> x >> y >> z;

		// since we are in 2D, we ignore z.
		points.emplace_back(x, y);

	}
	// cout << "parsed point file" << endl;
	return points;
}

Polygon load_obj(const std::string &filename) {

	std::ifstream in(filename);
	if (!in.is_open()) {
		throw std::runtime_error("Failed to open file " + filename);
	}

	Polygon poly;
	std::string line;

	while (std::getline(in, line)) {
        // Trim whitespace from the start for easier checking
        // line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](unsigned char ch) {
        //     return !std::isspace(ch);
        // }));

        if (line.rfind("v ", 0) == 0) {  // Check if the line starts with 'v '
            std::stringstream ss(line.substr(2));
            double x, y, z;
            ss >> x >> y >> z;  // Since we are in 2D, we can ignore z later.
            poly.emplace_back(x, y);
        }
        // Ignore comment lines
        else if (line.rfind("#", 0) == 0) {
            continue;
        }
    }

	// cout << "parsed polygon file" << endl;

	return poly;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	
	std::ofstream out(filename);
	
	if (!out.is_open()) {
		throw std::runtime_error("Failed to open file " + filename);
	}

	out << points.size() << "\n";
	for (const Point &p : points) {
		out << p.real() << " " << p.imag() << " " << 0.0 << "\n";
	}
}

////////////////////////////////////////////////////////////////////////////////
// MAIN: 

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}

	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);

	// result => only points inside poly.
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		cout << "iteration " << i << endl;
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
