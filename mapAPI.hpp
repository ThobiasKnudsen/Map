#include "DBG.hpp"

#include <set>
#include <string>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <filesystem>
#include <Windows.h>
#include <cmath>
#include <iostream>
#include <algorithm>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"   // ignorerer "-Weffc++" advarsler for disse under kompilasjonen
#include <GeographicLib/MGRS.hpp>         // for MGRS
#include <GeographicLib/UTMUPS.hpp>     // for MGRS
#include <GeographicLib/Geodesic.hpp> // for MGRS
#include <fmt/core.h>               // for printing
#pragma GCC diagnostic pop

using namespace std;
namespace fs = filesystem; // forkortelse for filesystem fra #include <filesystem>

const double EARTH_RADIUS_M = 6371000.f; // Jordas radius i kilometer

string replaceSubstring(const string& original, const string& toReplace, const string& replaceWith) {
    string modifiedString = original;
    // Find the start position of the substring we want to replace
    size_t startPos = modifiedString.find(toReplace);
    // Replace all occurrences of the substring
    while (startPos != string::npos) {
        modifiedString.replace(startPos, toReplace.length(), replaceWith);
        startPos = modifiedString.find(toReplace, startPos + replaceWith.length());
    }
    return modifiedString;
}

namespace MGRS {
	static string latLonToMGRS(double latitude, double longitude) {
		int zone;
		bool northp;
		double x, y;
		GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y);

		string mgrs;
		GeographicLib::MGRS::Forward(zone, northp, x, y, latitude, 5, mgrs);
		return mgrs;
	}
	static string MGRStoLatLon(string mgrs) {}
};

namespace Math {
    template<typename T>
    struct Rect {
    // |    TYPE   |    NAME    |        
        T           x1, y1, x2, y2;
                    Rect() : x1(T()), y1(T()), x2(T()), y2(T()) {}
                    Rect(T x1_in, T y1_in, T x2_in, T y2_in) : x1(x1_in), y1(y1_in), x2(x2_in), y2(y2_in) {}
        void        print() const {
			fmt::print("x1={}, y1={}, x2={}, y2={}\n", x1, y1, x2, y2);
		}
        string getInfo() const {
			return fmt::format("x1={}, y1={}, x2={}, y2={}", x1, y1, x2, y2);
		}
        T           w() const { return x2 - x1; }
        T           h() const { return y2 - y1; }
        bool        doRectsOverlap(const Math::Rect<T>& otherRect) const {
			if (x2 < otherRect.x1 || otherRect.x2 < x1) return false;
			if (y1 > otherRect.y2 || otherRect.y1 > y2) return false;
			return true;
		}
        void        zoom(float factor, T x, T y) {
			x += x1;
			y += y1;
			

			T x1_x_div_factor = static_cast<T>(static_cast<float>(x - x1) / factor);
			T y1_y_div_factor = static_cast<T>(static_cast<float>(y - y1) / factor);
			T x_x2_div_factor = static_cast<T>(static_cast<float>(x2 - x) / factor);
			T y_y2_div_factor = static_cast<T>(static_cast<float>(y2 - y) / factor);

			x1 = x - x1_x_div_factor;
			y1 = y - y1_y_div_factor;
			x2 = x + x_x2_div_factor;
			y2 = y + y_y2_div_factor;
		}
        void        setPos(T x, T y) {
			T width = w();
			x1 = x;
			x2 = x + width;
			T height = h();
			y1 = y;
			y2 = y + height;
		}
        void        setX(T x) {
			T width = w();
			x1 = x;
			x2 = x + width;
		}
        void        setY(T y) { 
			T height = h();
			y1 = y;
			y2 = y + height;
		}
    };
    struct Point {
        double x, y;
               Point() : x(-1.f), y(-1.f) {};
               Point(double x_in, double y_in) : x(x_in), y(y_in) {}
        double length() const {return sqrt(x*x+y*y);}
        void   setFromPolar(double angleRadians, double length) {
			x = length * cos(angleRadians);
			y = length * sin(angleRadians);
		}
        // Operator Overloads
        Point  operator+(const Point& other) const {return Point(x + other.x, y + other.y);}
        Point  operator+(double num) const {return Point(x + num, y + num);}
        Point  operator-(const Point& other) const {return Point(x - other.x, y - other.y);}
        Point  operator-(double num) const {return Point(x - num, y - num);}
        Point  operator*(const Point& other) const {return Point(x * other.x, y * other.y);}
        Point  operator*(double num) const {return Point(x * num, y * num);}
        Point  operator/(const Point& other) const {return Point(x / other.x, y / other.y);}
        Point  operator/(double num) const {return Point(x / num, y / num);}
        bool   operator==(double num) const {return x==num && y==num;}
        bool   operator!=(double num) const {return x!=num && y!=num;}
    };

    static double getAngleRadians(const double& v1x,const double& v1y, const double& v2x,const double& v2y) {
		double dot = v1x * v2x + v1y * v2y; // Dot product
		double det = v1x * v2y - v1y * v2x; // Determinant
		return atan2(det, dot); // atan2 returns the angle between the vectors
	}
    // in radians
    static float getAngleRadians(const float& v1x, const float& v1y, const float& v2x, const float& v2y) {
		float dot = v1x * v2x + v1y * v2y; // Dot product
		float det = v1x * v2y - v1y * v2x; // Determinant
		return atan2(det, dot); // atan2 returns the angle between the vectors
	}
    static double crossProduct(const Math::Point& p1, const Math::Point& p2, const Math::Point& p3){
		return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
	}
    static bool isPointInTriangle(const Math::Point& pt, const Math::Point& v1, const Math::Point& v2, const Math::Point& v3) {
		// Check cross product signs for each triangle formed with two vertices and the point
		bool b1, b2, b3;

		b1 = Math::crossProduct(pt, v1, v2) < 0.0f;
		b2 = Math::crossProduct(pt, v2, v3) < 0.0f;
		b3 = Math::crossProduct(pt, v3, v1) < 0.0f;

		// Check if point is on the same side of each edge
		return ((b1 == b2) && (b2 == b3));
	}

    static double toRadians(const double& degrees) {
		return degrees * (M_PI / 180.0);
	}
    static double haversineDistance(const Math::Point& p1, const Math::Point& p2) {
		// Convert latitude and longitude from degrees to radians
		double lat1 = Math::toRadians(p1.x);
		double lon1 = Math::toRadians(p1.y);
		double lat2 = Math::toRadians(p2.x);
		double lon2 = Math::toRadians(p2.y);

		// Haversine formula
		double dLat = lat2 - lat1;
		double dLon = lon2 - lon1;
		double a = pow(sin(dLat / 2), 2) +
				cos(lat1) * cos(lat2) * 
				pow(sin(dLon / 2), 2);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));
		//printf("dLat=%f dLon=%f return=%f ehh=%f\n", dLat, dLon, EARTH_RADIUS_M * c, c/M_PI);
		return EARTH_RADIUS_M * c;
	}
    static void latLonToXYZ(const double& lat, const double& lon, double& x, double& y, double& z) {
		double latRad = lat * (M_PI / 180.0);
		double lonRad = lon * (M_PI / 180.0);
		x = cos(latRad) * cos(lonRad);
		y = cos(latRad) * sin(lonRad);
		z = sin(latRad);
	}
    static void xyzToLatLon(const double& x, const double& y, const double& z, double& lat, double& lon) {
		lat = asin(z) * (180.0 / M_PI);
		lon = atan2(y, x) * (180.0 / M_PI);
	}
	static void crossProduct(const double& Ax, const double& Ay, const double& Az, const double& Bx, const double& By, const double& Bz, double& x, double& y, double& z) {
		x = Ay * Bz - Az * By;
		y = Az * Bx - Ax * Bz;
		z = Ax * By - Ay * Bx;
	}
    static void normalize(const double& Px, const double& Py, const double& Pz, double& x, double& y, double& z) {
		double norm = sqrt(Px * Px + Py * Py + Pz * Pz);
		x = Px / norm;
		y = Py / norm; 
		z = Pz / norm;
	}
    static void normalize(double& x, double& y) {
		double magnitude = sqrt(x * x + y * y);
		if (magnitude == 0) {
			DBG::Scope scope(__LINE__, __func__, __FILE__);
			DBG::warning(__LINE__, fmt::format("could not normalize vecor because magnitude is 0"));
			return;
		}
		x /= magnitude;
		y /= magnitude;
	}
	static double dot(const double& Vx1, const double& Vy1, const double& Vz1, const double& Vx2, const double& Vy2, const double& Vz2) {
		return Vx1 * Vx2 + Vy1 * Vy2 + Vz1 * Vz2;
	}
    static double magnitude(const double& Vx, const double& Vy, const double& Vz) {
		return sqrt(Vx * Vx + Vy * Vy + Vz * Vz);
	}
	static double angleBetweenVectors(const double& Vx1, const double& Vy1, const double& Vz1, const double& Vx2, const double& Vy2, const double& Vz2) {
		double dotProduct = Math::dot(Vx1, Vy1, Vz1, Vx2, Vy2, Vz2);
		double magV1 = Math::magnitude(Vx1, Vy1, Vz1);
		double magV2 = Math::magnitude(Vx2, Vy2, Vz2);
		// Calculate the cosine of the angle
		double cosAngle = dotProduct / (magV1 * magV2);
		// Ensuring the cosine value is within -1 to 1 range to avoid NaN due to floating point errors
		cosAngle = max(-1.0, min(1.0, cosAngle));
		// Calculate the angle in radians and then convert to degrees
		double angle = acos(cosAngle); // angle in radians
		return angle * (180.0 / M_PI);
	}
    static double angleBetweenThreeLatlonPos(const double& lat1, const double& lon1, const double& lat2, const double& lon2, const double& lat3, const double& lon3) {
		double Px1,Py1,Pz1,  Px2,Py2,Pz2,  Px3,Py3,Pz3;
		Math::latLonToXYZ(lat1,lon1,  Px1,Py1,Pz1);
		Math::latLonToXYZ(lat2,lon2,  Px2,Py2,Pz2);
		Math::latLonToXYZ(lat3,lon3,  Px3,Py3,Pz3);

		double Vx1,Vy1,Vz1,  Vx2,Vy2,Vz2;
		Vx1=Px1-Px2;  Vy1=Py1-Py2;  Vz1=Pz1-Pz2;
		Vx2=Px3-Px2;  Vy2=Py3-Py2;  Vz2=Pz3-Pz2;

		return Math::angleBetweenVectors(Vx1,Vy1,Vz1,  Vx2,Vy2,Vz2);
	}
	static double angleBetweenVectorsClockDirection(const double& Vx1, const double& Vy1, const double& Vz1, const double& Vx2, const double& Vy2, const double& Vz2, const double& PosX, const double& PosY, const double& PosZ) {
		double x1,y1,z1;
		Math::crossProduct(Vx1,Vy1,Vz1,  PosX, PosY, PosZ,  x1,y1,z1);
		Math::normalize(x1,y1,z1,  x1,y1,z1);

		double angle = Math::angleBetweenVectors(Vx1,Vy1,Vz1,  Vx2,Vy2,Vz2);
		bool angleGreaterThan180 = 90 > Math::angleBetweenVectors(Vx2,Vy2,Vz2,  x1,y1,z1) ? true : false;
		return angleGreaterThan180 ? 360.f-angle : angle;
	}
    static double angleBetweenThreeLatlonPosClockDirection(const double& lat1, const double& lon1, const double& lat2, const double& lon2, const double& lat3, const double& lon3) {
		double Px1,Py1,Pz1,  Px2,Py2,Pz2,  Px3,Py3,Pz3;
		Math::latLonToXYZ(lat1,lon1,  Px1,Py1,Pz1);
		Math::latLonToXYZ(lat2,lon2,  Px2,Py2,Pz2);
		Math::latLonToXYZ(lat3,lon3,  Px3,Py3,Pz3);

		double Vx1,Vy1,Vz1,  Vx2,Vy2,Vz2;
		Math::crossProduct(Px2,Py2,Pz2,  Px1,Py1,Pz1,  Vx1,Vy1,Vz1);
		Math::normalize(Vx1,Vy1,Vz1, Vx1,Vy1,Vz1);
		Math::crossProduct(Px2,Py2,Pz2,  Px3,Py3,Pz3,  Vx2,Vy2,Vz2);
		Math::normalize(Vx2,Vy2,Vz2, Vx2,Vy2,Vz2);

		return Math::angleBetweenVectorsClockDirection(Vx1,Vy1,Vz1,  Vx2,Vy2,Vz2,  Px2,Py2,Pz2);
	}
    static void slerp(double Ax, double Ay, double Az, double Bx, double By, double Bz, const double& t, double& x, double& y, double& z) {
		Math::normalize(Ax, Ay, Az, Ax, Ay, Az);
		Math::normalize(Bx, By, Bz, Bx, By, Bz);
		double omega = acos(Math::dot(Ax, Ay, Az, Bx, By, Bz));
		double sinOmega = sin(omega);
		x = (sin((1 - t) * omega) / sinOmega) * Ax + (sin(t * omega) / sinOmega) * Bx;
		y = (sin((1 - t) * omega) / sinOmega) * Ay + (sin(t * omega) / sinOmega) * By;
		z = (sin((1 - t) * omega) / sinOmega) * Az + (sin(t * omega) / sinOmega) * Bz;
	}
    static bool isPointInsideQuad(const Math::Point p, const Math::Point p1, const Math::Point p2, const Math::Point p3, const Math::Point p4) {
		double x,y,z,  x1,y1,z1,  x2,y2,z2,  x3,y3,z3,  x4,y4,z4;
		Math::latLonToXYZ(p.x,p.y,    x,y,z);
		Math::latLonToXYZ(p1.x,p1.y,  x1,y1,z1);
		Math::latLonToXYZ(p2.x,p2.y,  x2,y2,z2);
		Math::latLonToXYZ(p3.x,p3.y,  x3,y3,z3);
		Math::latLonToXYZ(p4.x,p4.y,  x4,y4,z4);

		double x1_2,y1_2,z1_2,  x3_4,y3_4,z3_4,  x1_3,y1_3,z1_3,  x2_4,y2_4,z2_4;
		Math::crossProduct(x1,y1,z1,  x2,y2,z2,  x1_2,y1_2,z1_2);
		Math::crossProduct(x3,y3,z3,  x4,y4,z4,  x3_4,y3_4,z3_4);
		Math::crossProduct(x1,y1,z1,  x3,y3,z3,  x1_3,y1_3,z1_3);
		Math::crossProduct(x2,y2,z2,  x4,y4,z4,  x2_4,y2_4,z2_4);

		double margin = 0;

		return (x1_2*x+y1_2*y+z1_2*z<=margin) and (x3_4*x+y3_4*y+z3_4*z>=-margin) and (x1_3*x+y1_3*y+z1_3*z>=-margin) and (x2_4*x+y2_4*y+z2_4*z<=margin);
	}
    static void rotateClockwise(const double x, const double y, const double angle, double& xOut, double& yOut) {
		double radians = angle * M_PI / 180.0;
		xOut = x * cos(radians) + y * sin(radians);
		yOut = -x * sin(radians) + y * cos(radians);
	}
    static void setVectorLength(double& x, double& y, const double newLength) {
		normalize(x, y);
		x *= newLength;
		y *= newLength;
	}
};

namespace Image {
    static float getSteepness_2x2(const vector<unsigned short>& vec, const int dataWidth, const int dataHeight, const int x, const int y, const float metersPerPixel, const int maxMountainHeight=1024) {
		if (vec.size() != static_cast<size_t>(dataWidth) * dataHeight || x + 1 >= dataWidth || y + 1 >= dataHeight) {
			return 0.0f;
		}

		// Directly calculate elevation differences and steepness
		float dx = abs(static_cast<float>(vec[x + 1 + y * dataWidth]) - vec[x + y * dataWidth]);
		float dy = abs(static_cast<float>(vec[x + (y + 1) * dataWidth]) - vec[x + y * dataWidth]);
		float diag = abs(static_cast<float>(vec[x + 1 + (y + 1) * dataWidth]) - vec[x + y * dataWidth]);
		
		// Normalize the differences by metersPerPixel and calculate the average steepness
		float steepnessX = dx / metersPerPixel;
		float steepnessY = dy / metersPerPixel;
		float steepnessDiag = diag / (static_cast<float>(M_SQRT2) * metersPerPixel);
		float averageSteepness = (steepnessX + steepnessY + steepnessDiag) / 3.0f;

		// Adjust the steepness calculation to fit within the expected range
		float factor = 65536.0f / static_cast<float>(maxMountainHeight);
		float normalizedSteepness = averageSteepness / factor;
		
		// Return the steepness as a ratio of the maximum possible angle (using atan for angle calculation)
		return atanf(normalizedSteepness) / (static_cast<float>(M_PI) / 2.0f);
	}
	static unsigned char getHeightLine_2x2(const vector<unsigned short>& vec, const int dataWidth, const int dataHeight, const int x, const int y, const unsigned short distBetweenLines, const int maxMountainHeight=1024) { 
		if (vec.size() != static_cast<size_t>(dataWidth) * dataHeight || x + 1 >= dataWidth || y + 1 >= dataHeight) {
			return 0;
		}

		unsigned short heightFactor = static_cast<unsigned short>(65536 / maxMountainHeight);
		unsigned short x0y0 = (vec[x     + y*dataWidth]    /heightFactor)/distBetweenLines;
		unsigned short x1y0 = (vec[(x+1) + y*dataWidth]    /heightFactor)/distBetweenLines;
		unsigned short x0y1 = (vec[x     + (y+1)*dataWidth]/heightFactor)/distBetweenLines;
		unsigned short x1y1 = (vec[(x+1) + (y+1)*dataWidth]/heightFactor)/distBetweenLines;

		unsigned char minVal = static_cast<unsigned char>(min({x0y0, x1y0, x0y1, x1y1}));
		unsigned char maxVal = static_cast<unsigned char>(max({x0y0, x1y0, x0y1, x1y1}));
		
		return maxVal-minVal;
	}
    static float getSunShadow_2x2(const vector<unsigned short>& vec, const int dataWidth, const int dataHeight, const int x, const int y, const float sunRayDirectionX, const float sunRayDirectionY) {
		if (vec.size() != static_cast<size_t>(dataWidth * dataHeight) || x + 1 >= dataWidth || y + 1 >= dataHeight) {
			return 0.0f;
		}
		// Directly compute elevation differences for the 2x2 area
		float diffX = static_cast<float>(vec[x + 1 + y * dataWidth] + vec[x + 1 + (y + 1) * dataWidth] - vec[x + y * dataWidth] - vec[x + (y + 1) * dataWidth]);
		float diffY = static_cast<float>(vec[x + (y + 1) * dataWidth] + vec[x + 1 + (y + 1) * dataWidth] - vec[x + y * dataWidth] - vec[x + 1 + y * dataWidth]);

		// Normalize upHillDirection to ensure it's a unit vector
		float magnitude = sqrtf(diffX * diffX + diffY * diffY);
		return magnitude>0.f ? cos(Math::getAngleRadians(diffX/magnitude, diffY/magnitude, sunRayDirectionX, sunRayDirectionY)) : 0.f;
	}
	static void getCustomColor2(unsigned char* red, unsigned char* green, unsigned char* blue, const float steepness, float sunShadow, const float terrainHeight_0to1, unsigned char treeHeight, const unsigned char maxTreeHeight=8, const float treeAlpha=0.7f) {
		
		sunShadow = (sunShadow+1.f)/2.f; // from -1 to 1 range to 0 to 0.5 range
		sunShadow*=sunShadow;

		static float white = 0.01f;
		static float yellow_red = 0.20f;
		static float red_black = 0.6f;
		static float no_green = 0.35f;

		if (steepness < white) 
		{
			*red  =static_cast<unsigned char>(192.f*terrainHeight_0to1);
			*green=static_cast<unsigned char>(192.f*terrainHeight_0to1);
			*blue =static_cast<unsigned char>(192.f*terrainHeight_0to1);
		}
		
		else if (steepness < yellow_red) 
		{
			*red  =static_cast<unsigned char>(192.f*(sunShadow+terrainHeight_0to1)/2.f);
			*green=static_cast<unsigned char>(192.f*(sunShadow+terrainHeight_0to1)/2.f);
			*blue =static_cast<unsigned char>(192.f*(sunShadow+terrainHeight_0to1)/2.f);
			
			*red  =static_cast<unsigned char>((steepness/yellow_red)*(255.f - static_cast<float>(*red)) + (*red));
			*green=static_cast<unsigned char>((steepness/0.5f)      *(255.f - static_cast<float>(*green)) + (*green));
		}

		else if ( steepness < red_black) 
		{
			*red  =255;
			*green=static_cast<unsigned char>(200.f*(1.f-((steepness-yellow_red)*(1.f/yellow_red))));
			*blue =0;
		}

		else if ( steepness < 1.f) 
		{
			*red  =static_cast<unsigned char>(255.f*(1.f-((steepness-red_black)*2)));
			*green=0;
			*blue =0;
		}

		treeHeight = (treeHeight>maxTreeHeight) ? maxTreeHeight : treeHeight;
		if (treeHeight != 0 and !( no_green <= steepness and steepness < 1)) {
			float height_0to1 = 0.2f*static_cast<float>(treeHeight)/static_cast<float>(maxTreeHeight);
			unsigned char newGreen = static_cast<unsigned char>(height_0to1*static_cast<float>(255-(*green)) + (*green));
			*green = static_cast<unsigned char>(treeAlpha*static_cast<float>(newGreen) + (1.f-treeAlpha)*static_cast<float>(*green));
			*red=(*red)/2;
			*blue=(*blue)/2;
		}
	}
    static void getCustomColor4(unsigned char* red, unsigned char* green, unsigned char* blue, const float steepness, const float sunShadow, const float terrainHeight, const unsigned char heightLine, unsigned char treeHeight, const unsigned char maxTreeHeight=8, const float treeAlpha=0.7f) {
		//float s = steepness;
		float ss = (sunShadow+1.f)/2.f; // from 0-1 range to 0.5-1 range
		float h = terrainHeight;

		if (steepness < 0.01f and heightLine>0) 
		{
			*red   = static_cast<unsigned char>(255.f*h); 
			*green = static_cast<unsigned char>(255.f*h); 
			*blue  = static_cast<unsigned char>(255.f*h);
		}

		else if (steepness < 0.01f and heightLine==0) 
		{
			*red   = static_cast<unsigned char>(128.f*h);
			*green = static_cast<unsigned char>(128.f*h); 
			*blue  = static_cast<unsigned char>(128.f*h);
		}
		
		else if (heightLine>0)
		{
			*red   = static_cast<unsigned char>(255.f*(ss+h)/2.f); 
			*green = static_cast<unsigned char>(255.f*(ss+h)/2.f); 
			*blue  = static_cast<unsigned char>(255.f*(ss+h)/2.f);
		}

		else if (heightLine==0)
		{
			*red   = static_cast<unsigned char>(128.f*(ss+h)/2.f); 
			*green = static_cast<unsigned char>(128.f*(ss+h)/2.f); 
			*blue  = static_cast<unsigned char>(128.f*(ss+h)/2.f);
		}

		treeHeight = (treeHeight>maxTreeHeight) ? maxTreeHeight : treeHeight;
		if (treeHeight != 0 and !( 0.6f <= steepness and steepness < 1)) {
			float treeHeight_0to1 = treeAlpha*static_cast<float>(treeHeight)/static_cast<float>(maxTreeHeight);
			unsigned char newGreen = static_cast<unsigned char>(treeHeight_0to1*static_cast<float>(255-(*green)) + (*green));
			*green = static_cast<unsigned char>(treeAlpha*static_cast<float>(newGreen) + (1.f-treeAlpha)*static_cast<float>(*green));
			*red/=2;
			*blue/=2;
		}
	}
    static float getHeight_0to1(const vector<unsigned short>& vec, const int dataWidth, const unsigned short lowest, const unsigned short highest, const int x, const int y){
		return (static_cast<float>(vec[x+y*dataWidth])-static_cast<float>(lowest))/(static_cast<float>(highest)-static_cast<float>(lowest));
	} 
};

struct Metadata {
    int                       widthPixels;
    int                       heightPixels;
    int                       metersPerPixel;
    int                       maxMountainHeight;
    Math::Point               p1 = {-1, -1}; 
    Math::Point               p2 = {-1, -1}; 
    Math::Point               p3 = {-1, -1};
    Math::Point               p4 = {-1, -1};
    string               filePath;

                              Metadata() : widthPixels(-1), heightPixels(-1), metersPerPixel(-1), maxMountainHeight(-1), p1(),p2(),p3(),p4(),filePath("") {}
                              Metadata(const string& filePath_in) : widthPixels(-1), heightPixels(-1), metersPerPixel(-1), maxMountainHeight(-1),  p1(), p2(), p3(), p4(), filePath(filePath_in) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		init(filePath_in);
	}
	void                      init(const string& filePath_in)  {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (filePath_in.substr(filePath_in.size() - 13) != "_metadata.txt") 
		{
			DBG::note(__LINE__, "given filePath doesn't end with '_metadata.txt'");
			DBG::note(__LINE__, filePath_in.c_str());
			DBG::warning(__LINE__);
		}

		if (!(fs::exists(fs::path(filePath_in)))) 
		{
			DBG::note(__LINE__, "WARNING given filePath doesn't exists\n");
			DBG::note(__LINE__, filePath_in.c_str());
			DBG::exit(__LINE__);        
		}

		this->filePath = filePath_in;
		this->readMetadataFile();
	}
    void                      readMetadataFile() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		ifstream file(filePath);
		string line;
		vector<string> lines;
		while (getline(file, line)) 
		{
			lines.push_back(line);
		}

		for (unsigned int i = 0; i < lines.size(); ++i)
		{
			if (lines[i].find('=') == string::npos)
			{
				DBG::warning(__LINE__, "'=' is not in line "+to_string(i)+"whole line: "+lines[i]);
				continue;
			} 

			istringstream iss(lines[i]);
			string key, value;
			if (!getline(iss, key, '=') || !getline(iss, value)) 
			{
				DBG::warning(__LINE__, "could not get key and value in line "+to_string(i));
				continue;
			}

			if (key.find("byte width") != string::npos) 
			{
				widthPixels = static_cast<int>(stof(value));
				DBG::note(__LINE__, fmt::format("widthPixels={}", widthPixels));
			} 
			else if (key.find("byte height") != string::npos) 
			{
				heightPixels = static_cast<int>(stof(value));
			}
			else if (key.find("max mountain height") != string::npos) 
			{
				maxMountainHeight = static_cast<int>(stof(value));
			} 
			else if (key.find("lat and lon") != string::npos)
			{
				istringstream latlonstream(value);
				string lat="", lon="";
				if (!getline(latlonstream, lat, ',') || !getline(latlonstream, lon)) 
				{
					DBG::warning(__LINE__, "could not get lat lon values in "+lines[i]);
					continue;
				}
				switch (i) 
				{
					case 3: p1.x = stof(lat); p1.y = stof(lon); break;
					case 4: p2.x = stof(lat); p2.y = stof(lon); break;
					case 5: p3.x = stof(lat); p3.y = stof(lon); break;
					case 6: p4.x = stof(lat); p4.y = stof(lon); break;
				}
			}
		}
		if (this->metersPerPixel == -1) {this->metersPerPixel = 1;}
		if (!isOK()) {
			DBG::error(__LINE__, fmt::format("metadata is not ok"));
		}
	}
	bool                      isOK() const {
		bool intsGood = widthPixels!=-1 && heightPixels!=-1 && metersPerPixel!=-1 && maxMountainHeight!=-1;
		bool pointsGood = p1!=-1 && p2!=-1 && p3!=-1 && p4!=-1; 
		return intsGood&&pointsGood;
	}
    void                      print() const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		DBG::note(__LINE__, "\n"+ filePath);
		DBG::note(__LINE__, "widthPixels "+to_string(widthPixels));
		DBG::note(__LINE__, "heightPixels "+to_string(heightPixels));
		DBG::note(__LINE__, "maxMountainHeight "+to_string(maxMountainHeight));
		DBG::note(__LINE__, "{("+to_string(p1.y)+","+ to_string(p1.x)+"),("
								+to_string(p2.y)+","+to_string(p2.x)+"),("
								+to_string(p3.y)+","+to_string(p3.x)+"),("
								+to_string(p4.y)+","+to_string(p4.x)+")}");
		
		DBG::print(__LINE__);
	}
	size_t                    getRAMsize() const {
		return 3*static_cast<size_t>(this->widthPixels)*static_cast<size_t>(this->heightPixels);
	}
    bool                      operator==(const Metadata& other_rect) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (this->widthPixels != other_rect.widthPixels) { return false;}
		if (this->heightPixels != other_rect.heightPixels) { return false;}
		if (this->p1.x != other_rect.p1.x) { return false;}
		if (this->p1.y != other_rect.p1.y) { return false;}
		if (this->p2.x != other_rect.p2.x) { return false;}
		if (this->p2.y != other_rect.p2.y) { return false;}
		if (this->p3.x != other_rect.p3.x) { return false;}
		if (this->p3.y != other_rect.p3.y) { return false;}
		if (this->p4.x != other_rect.p4.x) { return false;}
		if (this->p4.y != other_rect.p4.y) { return false;}
		 
		return true;
	}
	bool                      isLatLonPointWithin(Math::Point latLon) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		bool returnValue = Math::isPointInTriangle(latLon, p1, p2, p3) 
						or Math::isPointInTriangle(latLon, p2, p3, p4);
		
		return returnValue;
	}
    void                      uvToXYZ(double u, double v, double& x, double& y, double& z) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		double x1,y1,z1,  x2,y2,z2,  x3,y3,z3,  x4,y4,z4;
		Math::latLonToXYZ(p1.x,p1.y,  x1,y1,z1);
		Math::latLonToXYZ(p2.x,p2.y,  x2,y2,z2);
		Math::latLonToXYZ(p3.x,p3.y,  x3,y3,z3);
		Math::latLonToXYZ(p4.x,p4.y,  x4,y4,z4);

		double   x1_2,y1_2,z1_2,  x3_4,y3_4,z3_4;
		Math::slerp(x1,y1,z1,        x2,y2,z2,        u,  x1_2, y1_2, z1_2);
		Math::slerp(x3,y3,z3,        x4,y4,z4,        u,  x3_4, y3_4, z3_4);
		Math::slerp(x1_2,y1_2,z1_2,  x3_4,y3_4,z3_4,  v,  x,y,z);
	}
	void                      uvToLatLon(double u, double v, double& lat, double& lon) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		double x,y,z;
		uvToXYZ(u,v,x,y,z);
		Math::xyzToLatLon(x,y,z,lat,lon);
	}
    void                      pixelPosToLatLon(int x, int y, double& lat, double& lon) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		double u = (double)x/this->widthPixels;
		double v = (double)y/this->heightPixels;
		uvToLatLon(u,v,lat,lon);    
	}
	Math::Point               pixelPosToLatLon(int x, int y) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		double u = (double)x/this->widthPixels;
		double v = (double)y/this->heightPixels;
		Math::Point latLon;
		uvToLatLon(u,v,latLon.x,latLon.y);
		
		return latLon;
	}
    void                      meterPosToLatLon(int x, int y, double& lat, double& lon) const  {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		double u = (double)x/(this->widthPixels*this->metersPerPixel);
		double v = (double)y/(this->heightPixels*this->metersPerPixel);
		uvToLatLon(u,v,lat,lon);
	}
	bool                      isPointClose(Math::Point latLon, int metersOutsideXdir = 0, int metersOutsideYdir = 0) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		DBG::note(__LINE__, fmt::format("metersOutsideXdir={} metersOutsideYdir={}", metersOutsideXdir, metersOutsideYdir));
		int metersWidth = this->widthPixels*this->metersPerPixel;
		int metersHeight = this->heightPixels*this->metersPerPixel;
		DBG::note(__LINE__, fmt::format("metersWidth={} metersHeight={}", metersWidth, metersHeight));

		float uvXoutside = static_cast<float>(metersOutsideXdir)/static_cast<float>(metersWidth);
		float uvYoutside = static_cast<float>(metersOutsideYdir)/static_cast<float>(metersHeight);
		DBG::note(__LINE__, fmt::format("uvXoutside={} uvYoutside={}", uvXoutside, uvYoutside));

		Math::Point uvStart(-uvXoutside, -uvYoutside);
		Math::Point uvEnd(1.f+uvXoutside, 1.f+uvYoutside);

		DBG::note(__LINE__, fmt::format("start({}, {}) end({}, {})", uvStart.x, uvStart.y, uvEnd.x, uvEnd.y));
		if (uvStart.x>=uvEnd.x or uvStart.y>=uvEnd.y) {
			DBG::note(__LINE__, fmt::format("start({}, {}) is greater than end({}, {})", uvStart.x, uvStart.y, uvEnd.x, uvEnd.y));
			return false;
		}

		Math::Point latLon1,latLon2,latLon3,latLon4;
		uvToLatLon(uvStart.x, uvStart.y, latLon1.x, latLon1.y);
		uvToLatLon(uvEnd.x,   uvStart.y, latLon2.x, latLon2.y);
		uvToLatLon(uvStart.x, uvEnd.y,   latLon3.x, latLon3.y);
		uvToLatLon(uvEnd.x,   uvEnd.y,   latLon4.x, latLon4.y);

		//printf("{(%.10f,%.10f),(%.10f,%.10f),(%.10f,%.10f),(%.10f,%.10f)}\n", latLon1.y, latLon1.x, latLon2.y, latLon2.x, latLon3.y, latLon3.x, latLon4.y, latLon4.x);
		bool returnValue =  Math::isPointInsideQuad(latLon, latLon1, latLon2, latLon3, latLon4);
		
		DBG::note(__LINE__, fmt::format("point({:.5f}, {:.5f})", latLon.y, latLon.x));
		DBG::note(__LINE__, fmt::format("quad({}({:.5f}, {:.5f}), ({:.5f}, {:.5f}), ({:.5f}, {:.5f}), ({:.5f}, {:.5f}){})", "{",latLon1.y, latLon1.x, latLon2.y, latLon2.x, latLon3.y, latLon3.x, latLon4.y, latLon4.x, "}"));
		if (returnValue) {DBG::note(__LINE__, "return true;");}
		else {DBG::note(__LINE__, "return false;");}
		
		return returnValue;
	}
    // lon lat --> x y. if range is greater than 1 it will find x and y outside the subMap
    // e.g. range=3 makes the search area 9 (3^2) times larger than the map itself because
    void                      latLonToPixelXY(int&x, int&y, Math::Point latLon, int nItterations = 30, double range = 1.0f) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		//printf("    is (%f,%f) in {(%f,%f),(%f,%f),(%f,%f),(%f,%f)}\n", latLon.y, latLon.x, p1.y, p1.x, p2.y, p2.x, p3.y, p3.x, p4.y, p4.x);
		Math::Point tmp1, tmp2, tmp3, tmp4;
		double n = range/2;
		double start = 0.5f-n;
		Math::Point uv(start, start);

		for (int i = 0; i < nItterations; i++) {
			for (unsigned int j = 0; j <= 4; j++) {
				if (j==1 or j==3) {uv.x+=n;}
				else if (j==2) {uv.x-=n; uv.y+=n;}
				else if (j==4){
					DBG::warning(__LINE__, "could not get pixel x and y for given lon lat values");
					DBG::note(__LINE__, "   i="+to_string(i)+" n="+to_string(n)+" x="+to_string(uv.y)+" y="+to_string(uv.x));
					DBG::note(__LINE__, "   p="+to_string(latLon.y)+","+to_string(latLon.x));
					DBG::note(__LINE__, "   p1="+to_string(p1.y)+","+to_string(p1.x));
					DBG::note(__LINE__, "   p2="+to_string(p2.y)+",%f"+to_string(p2.x));
					DBG::note(__LINE__, "   p3="+to_string(p3.y)+","+to_string(p3.x));
					DBG::note(__LINE__, "   p4="+to_string(p4.y)+","+to_string(p4.x));
					uv=uv+(n/2);
					x = -static_cast<int>(uv.x*static_cast<double>(this->widthPixels));
					y = -static_cast<int>(uv.y*static_cast<double>(this->heightPixels));
					
					return;
				}

				//printf("        | j=%d n=%f x=%f y=%f", j, n, uv.x, uv.y);
				Math::Point AB1 = p1+(p2-p1)*(uv.x+start);
				Math::Point AB2 = p1+(p2-p1)*(uv.x+n);
				Math::Point CD1 = p3+(p4-p3)*(uv.x+start);
				Math::Point CD2 = p3+(p4-p3)*(uv.x+n);
				tmp1 = AB1 + (CD1 - AB1) * (uv.y+start);
				tmp2 = AB2 + (CD2 - AB2) * (uv.y+start);
				tmp3 = AB1 + (CD1 - AB1) * (uv.y+n);
				tmp4 = AB2 + (CD2 - AB2) * (uv.y+n);
				//printf("{(%f,%f),(%f,%f),(%f,%f),(%f,%f)}\n", AB1.y, AB1.x, AB2.y, AB2.x, CD1.y, CD1.x, CD2.y, CD2.x);
				if (Math::isPointInTriangle(latLon, tmp1, tmp2, tmp3) or Math::isPointInTriangle(latLon, tmp2, tmp3, tmp4)) {
					n/=2;
					//printf("    (%f,%f) in {(%f,%f),(%f,%f),(%f,%f),(%f,%f)}\n", latLon.y, latLon.x, tmp1.y, tmp1.x, tmp2.y, tmp2.x, tmp3.y, tmp3.x, tmp4.y, tmp4.x);
					//printf("        j=%d n=%f x=%f y=%f\n", j, n, uv.x, uv.y);
					//printf("  <--\n");
					break;
				}
				//printf("    (%f,%f) not in {(%f,%f),(%f,%f),(%f,%f),(%f,%f)}\n", latLon.y, latLon.x, tmp1.y, tmp1.x, tmp2.y, tmp2.x, tmp3.y, tmp3.x, tmp4.y, tmp4.x);
				//printf("\n");
			}
		}
		uv=uv+(n/2);
		x = -static_cast<int>(uv.x*static_cast<double>(this->widthPixels));
		y = -static_cast<int>(uv.y*static_cast<double>(this->heightPixels));
		//x/=this->metersPerPixel;
		//y/=this->metersPerPixel;
		
		return;
	}

    // lon lat --> x y. if range is greater than 1 it will find x and y outside the subMap
    // e.g. range=3 makes the search area 9 (3^2) times larger than the map itself because
    void                      latLonToMeterPos(int&x, int&y, Math::Point latLon, int nItterations = 30, int metersOutsideXdir = 0, int metersOutsideYdir = 0) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		//printf("    is (%f,%f) in {(%f,%f),(%f,%f),(%f,%f),(%f,%f)}\n", latLon.y, latLon.x, p1.y, p1.x, p2.y, p2.x, p3.y, p3.x, p4.y, p4.x);

		int metersWidth = this->widthPixels*this->metersPerPixel;
		int metersHeight = this->heightPixels*this->metersPerPixel;

		Math::Point uvStart(-((float)metersOutsideXdir/(float)metersWidth), -((float)metersOutsideYdir/(float)metersHeight));
		Math::Point uvEnd(1.f+((float)metersOutsideXdir/(float)metersWidth), 1.f+((float)metersOutsideYdir/(float)metersHeight));
		Math::Point uv = uvStart;
		Math::Point n((uvEnd.x-uvStart.x)/2, (uvEnd.y-uvStart.y)/2);

		//printf("uvStart(%f,%f) uvEnd(%f,%f) n(%f,%f)\n", uvStart.x, uvStart.y, uvEnd.x, uvEnd.y, n.x, n.y);

		if (uvStart.x>=uvEnd.x or uvStart.y>=uvEnd.y) 
		{
			return;
		}

		for (int i = 0; i < nItterations; i++) 
		{
			for (unsigned int j = 0; j <= 4; j++) 
			{
				if (j==1 or j==3) 
				{
					uv.x+=n.x;
				}
				else if (j==2) 
				{
					uv.x-=n.x; uv.y+=n.y;
				}
				else if (j==4)
				{
					uv=uv+(n/2);
					x = static_cast<int>(uv.x*static_cast<double>(this->widthPixels));
					y = static_cast<int>(uv.y*static_cast<double>(this->heightPixels));
					return;
				}

				Math::Point latLon1,latLon2,latLon3,latLon4;
				uvToLatLon(uv.x,     uv.y,     latLon1.x, latLon1.y);
				uvToLatLon(uv.x+n.x, uv.y,     latLon2.x, latLon2.y);
				uvToLatLon(uv.x,     uv.y+n.y, latLon3.x, latLon3.y);
				uvToLatLon(uv.x+n.x, uv.y+n.y, latLon4.x, latLon4.y);

				//printf("{(%.10f,%.10f),(%.10f,%.10f),(%.10f,%.10f),(%.10f,%.10f)}\n", latLon1.y, latLon1.x, latLon2.y, latLon2.x, latLon3.y, latLon3.x, latLon4.y, latLon4.x);
				if (Math::isPointInsideQuad(latLon, latLon1, latLon2, latLon3, latLon4)) 
				{
					n=n/2;
					break;
				}
			}
		}
		uv=uv+(n/2);
		x = static_cast<int>(uv.x*static_cast<double>(this->widthPixels*this->metersPerPixel));
		y = static_cast<int>(uv.y*static_cast<double>(this->heightPixels*this->metersPerPixel));
		
	}
	Math::Point               getNearestPoint(const Math::Point& P) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		Math::Point closestPoint = p1;
		if ((P-p2).length()<(P-closestPoint).length()) {closestPoint = p2;}
		if ((P-p3).length()<(P-closestPoint).length()) {closestPoint = p3;}
		if ((P-p4).length()<(P-closestPoint).length()) {closestPoint = p4;}
		//printf("%f %f\n", closestPoint.x, closestPoint.y);
		
		return closestPoint;
	}
    Math::Point				  getMiddleLatLonPoint() {
		Math::Point returnValue(0, 0);
		returnValue=returnValue+p1;
		returnValue=returnValue+p2;
		returnValue=returnValue+p3;
		returnValue=returnValue+p4;
		returnValue=returnValue/4;
		return returnValue;
	}
	double                    degreesNorth(int x1, 
                                           int y1,
                                           int x2, 
                                           int y2) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		Math::Point p1_local = pixelPosToLatLon(x1, y1);
		Math::Point p2_local = pixelPosToLatLon(x2, y2);
		Math::Point north(p1_local.x + 1.f, p1_local.y);
		if (north.x > 90) 
		{
			north.x = 90;
		}

		double Vx1,Vy1,Vz1,  Vx2,Vy2,Vz2,  Vx3,Vy3,Vz3;
		Math::latLonToXYZ(p1_local.x,p1_local.y,  Vx1,Vy1,Vz1);
		Math::latLonToXYZ(p2_local.x,p2_local.y,  Vx2,Vy2,Vz2);
		Math::latLonToXYZ(north.x,north.y,  Vx3,Vy3,Vz3);

		double Nx1,Ny1,Nz1,  Nx2,Ny2,Nz2;
		Math::crossProduct(Vx1,Vy1,Vz1,   Vx3,Vy3,Vz3,   Nx1,Ny1,Nz1);
		Math::crossProduct(Vx1,Vy1,Vz1,   Vx2,Vy2,Vz2,   Nx2,Ny2,Nz2);

		double returnValue = Math::angleBetweenVectorsClockDirection(Nx1,Ny1,Nz1,  Nx2,Ny2,Nz2,  Vx2,Vy2,Vz2);
		
		return returnValue;
	}

};

struct SubMap {
    Metadata                    	metadata;
    string                 			filePath;
    vector<unsigned short> 			DTM = {};
    vector<unsigned char>  			DOMminusDTM = {};
    int                         	metersPerPixel;
    int                         	skipFactor;
    int                         	xPlacementMeters;
    int                         	yPlacementMeters;

                                SubMap()  : metadata(), filePath(""),  DTM({}), DOMminusDTM({}), metersPerPixel(0),skipFactor(0), xPlacementMeters(0), yPlacementMeters(0) {}
                                SubMap(const string& filePath_in, int skipFactor_in = 0, int metersPerPixel_in = 1) : metadata(), filePath(filePath_in),DTM({}), DOMminusDTM({}), metersPerPixel(metersPerPixel_in), skipFactor(skipFactor_in), xPlacementMeters(0), yPlacementMeters(0) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		init(filePath, 1, skipFactor);
	}
    void                        init(string filePath_in, 
                                     unsigned int metersPerPixelMetadata = 1, 
                                     unsigned int skipFactor_in = 0) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (filePath_in.substr(filePath_in.length() - 8) == "_DTM.bin") {
			filePath_in = replaceSubstring(filePath_in, "_DTM.bin", "_metadata.txt");
		}
		else if (filePath_in.substr(filePath_in.length() - 16) == "_DOMminusDTM.bin") {
			filePath_in = replaceSubstring(filePath_in, "_DOMminusDTM.bin", "_metadata.txt");
		}
		else if (filePath_in.substr(filePath_in.length() - 13) != "_metadata.txt") {
			DBG::note(__LINE__, "filePath is not recognized");
			DBG::note(__LINE__, filePath_in.substr(filePath_in.length() - 13).c_str());
			DBG::error(__LINE__);
		}
		this->filePath = filePath_in;
		this->metadata.init(filePath);
		this->skipFactor = skipFactor_in;
		this->metersPerPixel = metersPerPixelMetadata*(skipFactor_in+1);
		readData();
	}
    void                        readData() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		ifstream DTM_file(replaceSubstring(filePath, "_metadata.txt","_DTM.bin"), ios::binary);
		ifstream DOMminusDTM_file(replaceSubstring(filePath, "_metadata.txt","_DOMminusDTM.bin"), ios::binary);
		if (!DTM_file or !DOMminusDTM_file) {
			DBG::error(__LINE__, "Could not open files");
		}

		// Determine the size of the file
		DTM_file.seekg(0, ios::end);
		size_t DTM_size = DTM_file.tellg();
		DTM_file.seekg(0, ios::beg);

		DOMminusDTM_file.seekg(0, ios::end);
		size_t DOMminusDTM_size = DOMminusDTM_file.tellg();
		DOMminusDTM_file.seekg(0, ios::beg);

		if (DTM_size != 2*DOMminusDTM_size) {
			DBG::error(__LINE__, "DTM is NOT 2x the size of DOMminusDTM\n"+filePath);
		}

		// Preallocate memory
		this->DTM.resize(DTM_size / 2);
		this->DOMminusDTM.resize(DOMminusDTM_size);

		if (DTM_size > 0 and DOMminusDTM_size > 0) {
			DTM_file.read(reinterpret_cast<char*>(&this->DTM[0]), DTM_size);
			DOMminusDTM_file.read(reinterpret_cast<char*>(&this->DOMminusDTM[0]), DOMminusDTM_size);
		}

		if (skipFactor >= 1) {
			vector<unsigned short> DTM_tmp((DTM_size / 2)/static_cast<size_t>(pow(skipFactor+1, 2)));
			vector<unsigned char> DOMminusDTM_tmp(DOMminusDTM_size/static_cast<size_t>(pow(skipFactor+1, 2)));

			unsigned int dist = skipFactor+1;
			unsigned int width = metadata.widthPixels/dist;
			unsigned int height = metadata.heightPixels/dist;

			#pragma omp parallel for collapse(2)
			for (unsigned int y = 0; y<height; y++) {
				for (unsigned int x = 0; x<width; x++) {
					DTM_tmp[x+y*width] = DTM[x*dist+y*dist*metadata.widthPixels];
					DOMminusDTM_tmp[x+y*width] = DOMminusDTM[x*dist+y*dist*metadata.widthPixels];
				}
			}

			this->DTM.swap(DTM_tmp);
			this->DOMminusDTM.swap(DOMminusDTM_tmp);
		} 

		DTM_file.close();
		DOMminusDTM_file.close();
		
	}
	size_t                      getRAMspace() const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		size_t returnValue = 2*DTM.size() + DOMminusDTM.size();
		return returnValue;
	}
    int                         getPixelWidth() const {
		return this->metadata.widthPixels/(this->skipFactor+1);
	}
    int                         getPixelHeight() const {
		return this->metadata.heightPixels/(this->skipFactor+1);
	}
    void                        print() const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		this->metadata.print();		
	}
    void                        copyFrameOfDTMandDOMminusDTMtoFrameOfDst(vector<unsigned short>& DTM_dst, vector<unsigned char>& DOMminusDTM_dst, const int dstWidth, const Math::Rect<int> applyToFrameOfDst, const Math::Rect<int> getFrameFromSrc) const {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (DTM_dst.size() != DOMminusDTM_dst.size()) 
		{
			DBG::note(__LINE__, "DTM_dst and DOMminusDTM_dst is not the same size");
			DBG::warning(__LINE__, "DTM and DOMminusDTM sizes "+
									to_string(DTM_dst.size())+" "+
									to_string(DOMminusDTM_dst.size()));
		}
		if (static_cast<int>(DTM_dst.size())%dstWidth != 0 or static_cast<int>(DOMminusDTM_dst.size())%dstWidth != 0) 
		{
			DBG::note(__LINE__, "DTM_dst or DOMminusDTM_dst is not a full rectangle");
			DBG::error(__LINE__, "    dstWidth= "+to_string(dstWidth)+
							" DTM_dst.size()= "+to_string(DTM_dst.size())+
							" DTM_dst.size()(percentage)dstWidth= "+to_string(static_cast<int>(DTM_dst.size())%dstWidth));
			return;
		}

		DBG::note(__LINE__, "applyToFrameOfDst: "+applyToFrameOfDst.getInfo());
		DBG::note(__LINE__, "getFrameFromSrc: "+getFrameFromSrc.getInfo());

		/*
		getFrameFromSrc.x1/=this->metersPerPixel;
		getFrameFromSrc.y1/=this->metersPerPixel;
		getFrameFromSrc.x2/=this->metersPerPixel;
		getFrameFromSrc.y2/=this->metersPerPixel;
		*/
		DBG::note(__LINE__, "DTM size = ("+
							to_string(static_cast<int>(this->DTM.size())/(this->metadata.heightPixels/this->metersPerPixel))+","+
							to_string(static_cast<int>(this->DTM.size())/(this->metadata.widthPixels/this->metersPerPixel))+")");

		int dstHeight = static_cast<int>(DTM_dst.size())/dstWidth;

		float dx = static_cast<float>(getFrameFromSrc.w())/(static_cast<float>(applyToFrameOfDst.w()));
		float dy = static_cast<float>(getFrameFromSrc.h())/(static_cast<float>(applyToFrameOfDst.h()));
		DBG::note(__LINE__, fmt::format("dx={} dy={}", dx, dy));

		// getting the part of the frame, applyToFrameOfDst, where the dst rect actually is
		Math::Rect<int> realFrameOfDst = applyToFrameOfDst; 
		if (realFrameOfDst.x1<0) {realFrameOfDst.x1 = 0;}
		if (realFrameOfDst.y1<0) {realFrameOfDst.y1 = 0;}
		if (realFrameOfDst.x2>dstWidth)  {realFrameOfDst.x2 = dstWidth;}
		if (realFrameOfDst.y2>dstHeight) {realFrameOfDst.y2 = dstHeight;}
		// this is done so that when getFrameFromSrc x1 or y1 is less than 0 it will move
		// realFrameOfDst rightwards and downwards
		if (getFrameFromSrc.x1<0) {realFrameOfDst.x1-=static_cast<int>(static_cast<float>(getFrameFromSrc.x1)/dx);}
		if (getFrameFromSrc.y1<0) {realFrameOfDst.y1-=static_cast<int>(static_cast<float>(getFrameFromSrc.y1)/dy);}
		
		if (realFrameOfDst.w()<=0 or realFrameOfDst.h()<=0) 
		{
			DBG::note(__LINE__, "realFrameOfDst.w()= "+to_string(realFrameOfDst.w())+" realFrameOfDst.h()= "+to_string(realFrameOfDst.h()));
			DBG::note(__LINE__, "did not write anything");
			return;
		}
		// error handling
		
		// offset which will be added to the x and y loop values when used for indexing in dst 
		// which is needed for when x and y in realFrameOfDst doesnt start on 0 
		int xOffsetDst = realFrameOfDst.x1;
		int yOffsetDst = realFrameOfDst.y1;
		DBG::note(__LINE__, fmt::format("xOffsetDst={} yOffsetDst={}", xOffsetDst, yOffsetDst));

		// getting the part of the frame, getFrameFromSrc, where this->buffer rect actually is
		Math::Rect<int> realFrameOfSrc = getFrameFromSrc;
		if (realFrameOfSrc.x1<0) {realFrameOfSrc.x1 = 0;}
		if (realFrameOfSrc.y1<0) {realFrameOfSrc.y1 = 0;}
		if (realFrameOfSrc.x2>this->metadata.widthPixels)  {realFrameOfSrc.x2 = this->metadata.widthPixels;}
		if (realFrameOfSrc.y2>this->metadata.heightPixels) {realFrameOfSrc.y2 = this->metadata.heightPixels;}
		// error handling
		if (realFrameOfSrc.w()<=0 or realFrameOfSrc.h()<=0) 
		{
			DBG::note(__LINE__, "realFrameOfSrc.w() or realFrameOfSrc.h() cant be less than or equal to 0");
			DBG::note(__LINE__, "realFrameOfSrc: "+ realFrameOfSrc.getInfo());
			DBG::note(__LINE__, "did not write anything");
			return;
		}
		
		// when getFrameFromSrc is larger than applyToFrameOfDst dx and dy will be greater than 1 and therefor 
		// some values in this->buffer is skipped. therefore this rect is needed to know the number of values 
		// actually extracted from this->buffer. it is also transformed by dividing by dy and dx to know where
		// the values should be extracted
		Math::Rect<float> realFrameOfSrcWithZoomOut;
		realFrameOfSrcWithZoomOut.x1=static_cast<float>(realFrameOfSrc.x1)/dx;
		realFrameOfSrcWithZoomOut.y1=static_cast<float>(realFrameOfSrc.y1)/dy;
		realFrameOfSrcWithZoomOut.x2=static_cast<float>(realFrameOfSrc.x2)/dx;
		realFrameOfSrcWithZoomOut.y2=static_cast<float>(realFrameOfSrc.y2)/dy;
		// this is done so that when applyToFrameOfDst x1 or y1 is less than 0 it will shift forwards
		// where this->buffer will start to be read
		if (applyToFrameOfDst.x1<0) 
		{
			realFrameOfSrcWithZoomOut.x1-=static_cast<float>(applyToFrameOfDst.x1)/static_cast<float>(this->metersPerPixel);
		}
		if (applyToFrameOfDst.y1<0) 
		{
			realFrameOfSrcWithZoomOut.y1-=static_cast<float>(applyToFrameOfDst.y1)/static_cast<float>(this->metersPerPixel);
		}
		// error handling
		if (realFrameOfSrcWithZoomOut.w()<=0 or realFrameOfSrcWithZoomOut.h()<=0) 
		{
			DBG::note(__LINE__, "realFrameOfSrcWithZoomOut.w() or realFrameOfSrcWithZoomOut.h() cant be less than or equal to 0");
			DBG::error(__LINE__, "    realFrameOfSrcWithZoomOut: "+ realFrameOfSrcWithZoomOut.getInfo());
			return;
		}

		// offset which will be added to the x and y loop values when used for indexing in this->buffer 
		// which is needed for when x and y in realFrameOfSrcWithZoomOut doesnt start on 0 
		float xOffsetSrc = realFrameOfSrcWithZoomOut.x1;                                 
		float yOffsetSrc = realFrameOfSrcWithZoomOut.y1;      

		DBG::note(__LINE__, fmt::format("xOffsetSrc={} yOffsetSrc={}", xOffsetSrc, yOffsetSrc));

		// gets the smallest width and height of realFrameOfDst and realFrameOfSrcWithZoomOut
		int width = realFrameOfDst.w();                                                   
		if (static_cast<float>(width) > realFrameOfSrcWithZoomOut.w()) {width = static_cast<int>(realFrameOfSrcWithZoomOut.w());}
		int height = realFrameOfDst.h();                                                    
		if (static_cast<float>(height) > realFrameOfSrcWithZoomOut.h()) {height = static_cast<int>(realFrameOfSrcWithZoomOut.h());}

		DBG::note(__LINE__, fmt::format("width={} height={}", width, height));
		
		// check if loop goes outside dst
		int dstSize = static_cast<int>(DTM_dst.size());
		int maxDstIndexLoop = ((width-1)+xOffsetDst) + ((height-2)+yOffsetDst)*dstWidth;
		if (dstSize<maxDstIndexLoop) {
			DBG::note(__LINE__, fmt::format("dstSize={} > maxDstIndexLoop={}", dstSize, maxDstIndexLoop));
			DBG::error(__LINE__, fmt::format("loop goes outside dst bounds"));
		}

		int pixelWidth = (this->metadata.widthPixels-(this->metadata.widthPixels%this->metersPerPixel))/this->metersPerPixel;
		DBG::note(__LINE__, fmt::format("pixelWidth={}",pixelWidth));

		//check if loop goes outside src bounds
		int srcSize = static_cast<int>(this->DTM.size());
		int maxXloop = static_cast<int>((static_cast<float>(width-1)+xOffsetSrc)*dx/static_cast<float>(this->metersPerPixel));
		int maxYloop = static_cast<int>((static_cast<float>(height-2)+yOffsetSrc)*dy/static_cast<float>(this->metersPerPixel));
		int maxSrcIndexLoop = maxXloop + maxYloop*pixelWidth;
		if (srcSize<maxSrcIndexLoop) {
			DBG::note(__LINE__, fmt::format("srcSize={} > maxSrcIndexLoop={}", srcSize, maxSrcIndexLoop));
			DBG::error(__LINE__, fmt::format("loop goes outside src bounds"));
		}
		
		#pragma omp parallel for collapse(2)
		for (int y = 0; y < height-1; ++y) 
		{
			for (int x = 0; x < width; ++x) 
			{
				int X = static_cast<int>((static_cast<float>(x)+xOffsetSrc)*dx/static_cast<float>(this->metersPerPixel));
				int Y = static_cast<int>((static_cast<float>(y)+yOffsetSrc)*dy/static_cast<float>(this->metersPerPixel));
				int indexSrc = X + Y*pixelWidth;
				int indexDst = (x+xOffsetDst) + (y+yOffsetDst)*dstWidth;
				//printf("X %d Y %d\n", X, Y);
				DTM_dst[indexDst] = this->DTM[indexSrc];
				DOMminusDTM_dst[indexDst] = this->DOMminusDTM[indexSrc];
			}
		}
	}
};

struct Map {
	string                 	dataFolderPath="";
	size_t                  maxRAM; // 1GB
	
    vector<SubMap>         	subMaps = {};
    vector<Metadata>       	allMetadata = {};
	Math::Point             currMidLatLonPos;
    int                     metersWidth, metersHeight;
	double                  degreesNorth; // between 0 and 360    
	
	vector<unsigned short> 	DTM = {};
    vector<unsigned char>  	DOMminusDTM = {};
	vector<unsigned char>  	rgbArray = {};
	int  					winWidth, winHeight;
	
	unsigned int 			mapType=0;
	
    Map(const string& dataFolderPath_in, double latitude, double longitude, int winWidth_in, int winHeight_in) : maxRAM(1000000000), metersWidth(20000), metersHeight(20000), degreesNorth(0), winWidth(winWidth_in), winHeight(winHeight_in) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		
		setDataFolderPath(dataFolderPath_in);
		currMidLatLonPos = Math::Point(latitude, longitude);
	}
	bool setDataFolderPath(string dataFolderPath_in) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		
		fs::path p(dataFolderPath_in);
		if (!(fs::exists(p) && fs::is_directory(p))) {
			return false;
		}
		if (this->dataFolderPath == dataFolderPath_in) {
			return true;
		}
		
		this->dataFolderPath = dataFolderPath_in;
		vector<string> allFilePaths={};
		for (const auto& entry : fs::directory_iterator(this->dataFolderPath)) {
			allFilePaths.push_back(entry.path().string());
		}
		allMetadata={};
		for (const auto& filePath : allFilePaths) {
			if (filePath.substr(filePath.size() - 13) == "_metadata.txt") {
				allMetadata.push_back(Metadata(filePath));
			}
		}
		DBG::note(__LINE__, fmt::format("added all {} metadata", allMetadata.size()));
		DBG::print();
		
		vector<Metadata> middleMetadata = getAllMetadataForPoint(currMidLatLonPos);
		if (middleMetadata.empty()) {
			vector<Metadata> middleMetadata = getAllVisibleMetadata();
		}
		if (middleMetadata.empty()) {
			if (this->allMetadata.empty()) {
				DBG::warning(__LINE__, "there is no valid data in given data folder path");
				return false;
			}
			middleMetadata.push_back(this->allMetadata[0]);
		}
		currMidLatLonPos = this->allMetadata[0].getMiddleLatLonPoint();
		degreesNorth = middleMetadata[0].degreesNorth(1, 1, 1, 0);
		DBG::note(__LINE__, fmt::format("degreesNorth = {}", degreesNorth));
		
		return true;
	}
    vector<Metadata> getAllMetadataForPoint(Math::Point p, int metersOutsideXdir = 0, int metersOutsideYdir = 0) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		vector<Metadata> returnValue = {};
		for (int i = 0; i < static_cast<int>(this->allMetadata.size()); i++) {
			if (this->allMetadata[i].isPointClose(p, metersOutsideXdir, metersOutsideYdir)) {
				returnValue.push_back(this->allMetadata[i]);
			}
		}    
		return returnValue;
	}
    vector<Metadata> getAllVisibleMetadata() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		vector<Metadata> returnValue = {};

		int metersOutsideXdir = static_cast<int>(static_cast<float>(this->metersWidth) *1.1f);
		int metersOutsideYdir = static_cast<int>(static_cast<float>(this->metersHeight)*1.1f);

		DBG::note(__LINE__,fmt::format("metersOutsideXdir = {} metersOutsideYdir = {}", metersOutsideXdir, metersOutsideYdir));

		for (int i = 0; i < static_cast<int>(this->allMetadata.size()); i++) {
			if (!(this->allMetadata[i].isPointClose(currMidLatLonPos, metersOutsideXdir/2, metersOutsideYdir/2))) {
				continue;
			}
			returnValue.push_back(this->allMetadata[i]);
		}
		return returnValue;
	}
    Math::Point getLatLonForPixel(const int x, const int y) const {
		Math::Point returnLatLon;
		
		int meterX = (metersWidth/winWidth)*x;
		int meterY = (metersHeight/winHeight)*y;
		
		
		
		for (auto subMap : this->subMaps) {
			
			int meterX1subMap = subMap.xPlacementMeters;
			int meterY1subMap = subMap.yPlacementMeters;
			int widthSubMap = subMap.getPixelWidth()*subMap.metersPerPixel;
			int heightSubMap = subMap.getPixelHeight()*subMap.metersPerPixel;
			
			
			
			if (meterX1subMap <= meterX and meterX < meterX1subMap+widthSubMap) {
				if (meterY1subMap <= meterY and meterY < meterY1subMap+heightSubMap) {
					int meterXposSubMap = -meterX1subMap + meterX;
					int meterYposSubMap = -meterY1subMap + meterY;
					
					subMap.metadata.meterPosToLatLon(meterXposSubMap, meterYposSubMap, returnLatLon.x, returnLatLon.y);
					break;
				}
			}
		}
		return returnLatLon;
	}
	void moveView(int dxPixel, int dyPixel) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		dxPixel=-dxPixel;
		dyPixel=-dyPixel;

		vector<Metadata> middleMetadatas = getAllMetadataForPoint(this->currMidLatLonPos);
		if (middleMetadatas.empty()) {
			vector<Metadata> middleMetadatas = getAllVisibleMetadata();
		}
		if (middleMetadatas.empty()) {
			if (this->allMetadata.empty()) {
				DBG::warning(__LINE__, "there is no valid data in given data folder path");
				return;
			}
			middleMetadatas.push_back(this->allMetadata[0]);
			currMidLatLonPos = this->allMetadata[0].getMiddleLatLonPoint();
		}

		double degreesNorthMiddleMetadata =  middleMetadatas[0].degreesNorth(1, 1, 1, 0);
		// from subMap up, to view up
		double degreesClockwiseBetween = this->degreesNorth-degreesNorthMiddleMetadata;
		degreesClockwiseBetween = degreesClockwiseBetween<0.f 
								? degreesClockwiseBetween+360.f 
								: degreesClockwiseBetween;
		
		// right direction for submap but not right length
		double xDirectionSubMap, yDirectionSubMap; 
		Math::rotateClockwise(dxPixel, dyPixel, degreesClockwiseBetween, xDirectionSubMap, yDirectionSubMap);

		double metersPerPixelX = static_cast<double>(this->metersWidth)/static_cast<double>(this->winWidth);
		double metersPerPixelY = static_cast<double>(this->metersHeight)/static_cast<double>(this->winHeight);
		double dx_meters = static_cast<double>(dxPixel)*metersPerPixelX;
		double dy_meters = static_cast<double>(dyPixel)*metersPerPixelY;
		double deltaDistance_meters = sqrt(dx_meters*dx_meters + dy_meters*dy_meters);

		// gets the new middle x y meters pos 
		double dxSubMap_meters=static_cast<double>(xDirectionSubMap);
		double dySubMap_meters=static_cast<double>(yDirectionSubMap);
		Math::setVectorLength(dxSubMap_meters, dySubMap_meters, deltaDistance_meters);

		int currMidViewSubMapMeterX, currMidViewSubMapMeterY;
		middleMetadatas[0].latLonToMeterPos(currMidViewSubMapMeterX, currMidViewSubMapMeterY, this->currMidLatLonPos);

		int newXsubMap=currMidViewSubMapMeterX + static_cast<int>(dxSubMap_meters);
		int newYsubMap=currMidViewSubMapMeterY + static_cast<int>(dySubMap_meters);

		DBG::note(__LINE__, fmt::format("{} subMaps found", middleMetadatas.size()));
		DBG::note(__LINE__, fmt::format("degreesNorthMiddleSubMapMetadat={}", degreesNorthMiddleMetadata));
		DBG::note(__LINE__, fmt::format("dxPixel={} dyPixel={}", dxPixel, dyPixel));
		DBG::note(__LINE__, fmt::format("dxPixel={} dyPixel={}", dxPixel, dyPixel));
		DBG::note(__LINE__, fmt::format("xDirectionSubMap={} yDirectionSubMap={}", xDirectionSubMap, yDirectionSubMap));
		DBG::note(__LINE__, fmt::format("dx_meters={} dy_meters={}", dx_meters, dy_meters));
		DBG::note(__LINE__, fmt::format("deltaDistance_meters={}", deltaDistance_meters));
		DBG::note(__LINE__, fmt::format("dxSubMap_meters={} dySubMap_meters={}", dxSubMap_meters, dySubMap_meters));
		DBG::note(__LINE__, fmt::format("currMidViewSubMapMeterX={} currMidViewSubMapMeterY={}", currMidViewSubMapMeterX, currMidViewSubMapMeterY));
		DBG::note(__LINE__, fmt::format("oldCurrLatLonPos({}, {})", this->currMidLatLonPos.x, this->currMidLatLonPos.y));
		middleMetadatas[0].meterPosToLatLon(newXsubMap, newYsubMap, this->currMidLatLonPos.x, this->currMidLatLonPos.y);
		DBG::note(__LINE__, fmt::format("newCurrLatLonPos({}, {})", this->currMidLatLonPos.x, this->currMidLatLonPos.y));
		//DBG::print(__LINE__);
		
		this->updateSubMaps();
		this->updateMap();
		this->terrainAndTreeDataToRGBarray();
	}
    void zoom(int xPixelMouse, int yPixelMouse, float factor) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		float x1_x_div_factor = static_cast<float>(xPixelMouse) / factor;
		float y1_y_div_factor = static_cast<float>(yPixelMouse) / factor;
		float x_x2_div_factor = static_cast<float>(this->winWidth - xPixelMouse) / factor;
		float y_y2_div_factor = static_cast<float>(this->winHeight - yPixelMouse) / factor;

		float tmpX1 = static_cast<float>(xPixelMouse) - x1_x_div_factor;
		float tmpY1 = static_cast<float>(yPixelMouse) - y1_y_div_factor;
		float tmpX2 = static_cast<float>(xPixelMouse) + x_x2_div_factor;
		float tmpY2 = static_cast<float>(yPixelMouse) + y_y2_div_factor;

		float widthFactor = (tmpX2-tmpX1)/static_cast<float>(this->winWidth);
		float heightFactor = (tmpY2-tmpY1)/static_cast<float>(this->winHeight);

		int dxPixel = static_cast<int>(static_cast<float>(tmpX1+tmpX2)/2.f)-(this->winWidth/2);
		int dyPixel = static_cast<int>(static_cast<float>(tmpY1+tmpY2)/2.f)-(this->winHeight/2);

		this->moveView(-dxPixel, -dyPixel);

		this->metersWidth = static_cast<int>(static_cast<float>(this->metersWidth) * widthFactor);
		this->metersHeight = static_cast<int>(static_cast<float>(this->metersHeight) * heightFactor);

		if (this->metersWidth<static_cast<int>(this->winWidth)) {
			this->metersWidth=static_cast<int>(this->winWidth);
		}
		if (this->metersHeight<static_cast<int>(this->winHeight)) {
			this->metersHeight=static_cast<int>(this->winHeight);
		}
		DBG::note(__LINE__, "functions finished");


		this->updateSubMaps();
		this->updateMap();
		this->terrainAndTreeDataToRGBarray();
	}
    void updateSubMaps() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		vector<Metadata> allVisibleMetadata = getAllVisibleMetadata();
		DBG::note(__LINE__, "found "+to_string(allVisibleMetadata.size())+" subMaps");

		/*
		for (auto metadata : allVisibleMetadata) {
			SubMap subMap(metadata.filePath);
			Math::Point latLon;
			subMap.metadata.pixelPosToLatLon(7505, 7505, latLon.x, latLon.y);
			printf("%f %f\n", latLon.x, latLon.y);
			printf("%s\n", metadata.filePath.c_str());
			subMap.show3(2, 2);
		}
		*/
		int metersPerPixel = this->metersWidth/winWidth;
		DBG::note(__LINE__, fmt::format("metersPerPixel for window={}", metersPerPixel));
		DBG::note(__LINE__, fmt::format("start erasing unsused submaps"));
		// erase submaps thats no longer in view
		for (int i = 0; i < static_cast<int>(this->subMaps.size()); i++) {
			bool subMapNoLongerVisible = true;
			for (Metadata visible : allVisibleMetadata) {
				if (visible.filePath == this->subMaps[i].filePath) {
					subMapNoLongerVisible = false;
				}
			}
			if (subMapNoLongerVisible) {
				string filePath = this->subMaps[i].filePath;
				this->subMaps.erase(this->subMaps.begin() + i);
				DBG::note(__LINE__, "erased old map thats no longer in view");
				DBG::note(__LINE__, "    "+filePath);
			}
		}

		DBG::note(__LINE__, fmt::format("re-evaluating resolution of submaps"));
		// re-evaluating skipFactor for previosly and currently visible maps
		for (int i = 0; i < static_cast<int>(this->subMaps.size()); i++) {
			int skipFactor = (metersPerPixel/subMaps[i].metadata.metersPerPixel) - 1;
			DBG::note(__LINE__, fmt::format("skipfactor for subMap[{}] = {}", i, skipFactor));
			if (this->subMaps[i].skipFactor != skipFactor) {
				this->subMaps[i].init(this->subMaps[i].filePath, 1, skipFactor);
				DBG::note(__LINE__, "resized subMap");
				DBG::note(__LINE__, "    "+this->subMaps[i].filePath);
			}
		}

		// adding the new maps that earlier wasnt in view
		DBG::note(__LINE__, fmt::format("adding new submaps"));
		for (Metadata visible : allVisibleMetadata) {
			int skipFactor = (metersPerPixel/visible.metersPerPixel) - 1;
			DBG::note(__LINE__, fmt::format("skipfactor for metadata[] = {}", skipFactor));
			bool newVisibleThatIsNotInPrev = true;
			for (int i = 0; i < static_cast<int>(this->subMaps.size()); i++) {
				if (visible.filePath == this->subMaps[i].filePath) {
					newVisibleThatIsNotInPrev = false;
				}
			}
			if (newVisibleThatIsNotInPrev) {
				this->subMaps.push_back(SubMap(visible.filePath, skipFactor));
				DBG::note(__LINE__, "added new map: "+visible.filePath);
			}
		}

		DBG::note(__LINE__, fmt::format("setting position of submaps"));
		// setting position for each subMap
		for (int i = 0; i < static_cast<int>(this->subMaps.size()); i++) {
			//this->currMidLatLonPos
			int metersOutsideXdir = this->metersWidth;
			int metersOutsideYdir = this->metersHeight;
			int x, y;

			this->subMaps[i].metadata.latLonToMeterPos(x, 
															 y, 
															 this->currMidLatLonPos, 
															 30, 
															 metersOutsideXdir, 
															 metersOutsideYdir);
			x=-x;
			y=-y;

			x += this->metersWidth/2;
			y += this->metersHeight/2;
			this->subMaps[i].xPlacementMeters = x;
			this->subMaps[i].yPlacementMeters = y;
			fmt::print("subMap: {} {} {} {} {} {}\n", x, 
													y, 
													x+subMaps[i].metadata.widthPixels*subMaps[i].metadata.metersPerPixel, 
													y+subMaps[i].metadata.heightPixels*subMaps[i].metadata.metersPerPixel,
													subMaps[i].metadata.widthPixels*subMaps[i].metadata.metersPerPixel, 
													subMaps[i].metadata.heightPixels*subMaps[i].metadata.metersPerPixel);
													
			Math::Point latLon;
			this->subMaps[i].metadata.pixelPosToLatLon(this->subMaps[i].metadata.widthPixels/2, 
															 this->subMaps[i].metadata.heightPixels/2, 
															 latLon.x, 
															 latLon.y);
		}
		DBG::note(__LINE__, fmt::format("functions finished"));
	}
    void updateMap() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		Math::Rect<int> mapFrameMeters(0, 0, this->metersWidth, this->metersHeight);
		Math::Rect<int> mapFramePixels(0, 0, this->winWidth,    this->winHeight);

		if (static_cast<int>(this->DTM.size()) != this->winWidth * this->winHeight) {            
			this->DTM.resize(this->winWidth * this->winHeight);
			this->DOMminusDTM.resize(this->winWidth * this->winHeight);
			DBG::note(__LINE__, fmt::format("resizing DTM and DOMminusDTM to {} {}", this->winWidth, this->winHeight));
		}

		DBG::note(__LINE__, fmt::format("{} subMaps.size()\n", subMaps.size()));

		// clearing data
		this->DTM.assign(DTM.size(), 0);
		this->DOMminusDTM.assign(DOMminusDTM.size(), 0);

		for (int i = 0; i < static_cast<int>(this->subMaps.size()); i++) {

			Math::Rect<int> subMapFramePixels = mapFrameMeters;
			DBG::note(__LINE__, fmt::format("subMaps[{}].DTM.size()={}",i, subMaps[i].DTM.size()));

			DBG::note(__LINE__, fmt::format("subMapFramePixels.x2={} subMapFramePixels.y2={}", subMapFramePixels.x2, subMapFramePixels.y2));

			// adjusts size from meters to pixels
			//subMapFramePixels.x2 /= (int)this->subMaps[i].metersPerPixel;
			//subMapFramePixels.y2 /= (int)this->subMaps[i].metersPerPixel;

			DBG::note(__LINE__, fmt::format("metersPerPixel.x2={}", this->subMaps[i].metersPerPixel));
			DBG::note(__LINE__, fmt::format("subMapFramePixels.x2={} subMapFramePixels.y2={}", subMapFramePixels.x2, subMapFramePixels.y2));

			// adjusts position
			//int xPos = -this->subMaps[i].xPlacementMeters / this->subMaps[i].metersPerPixel * 2;
			//int yPos = -this->subMaps[i].yPlacementMeters / this->subMaps[i].metersPerPixel * 2;
			int xPos = -this->subMaps[i].xPlacementMeters;
			int yPos = -this->subMaps[i].yPlacementMeters;
			subMapFramePixels.setPos(xPos, yPos);

			DBG::note(__LINE__, fmt::format("subMapFramePixels.w()={} subMapFramePixels.h()={} subMaps[i].metersPerPixel={}", subMapFramePixels.w(), subMapFramePixels.h(), subMaps[i].metersPerPixel));
			DBG::note(__LINE__, fmt::format("subMapFramePixels.x1={} subMapFramePixels.y1={}", subMapFramePixels.x1, subMapFramePixels.y1));

			// copy data
			DBG::note(__LINE__, fmt::format("topLeftPos({}, {})", subMaps[i].metadata.p1.y, subMaps[i].metadata.p1.x));
			DBG::note(__LINE__, fmt::format("mapFramePixels={}", mapFramePixels.getInfo()));
			DBG::note(__LINE__, fmt::format("subMapFramePixels={}\n", subMapFramePixels.getInfo()));


			this->subMaps[i].copyFrameOfDTMandDOMminusDTMtoFrameOfDst(this->DTM,
																	  this->DOMminusDTM,
																	  mapFramePixels.w(),
																	  mapFramePixels,
																	  subMapFramePixels);
		}
		if (this->subMaps.size() == 0) {
			DBG::warning(__LINE__, "0 subMaps.size()");
		}
	}
	void blueAreaToRGBarrayPtr(unsigned char* rgbArray_dst, int rgbArrayWidth_dst, int rgbArrayHeight_dst, int xPos, int yPos, float posElevation=2, float findAtElevation=0, int maxTerrainElevation=1024, float alpha=0.3f) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		
		if (rgbArrayWidth_dst*rgbArrayHeight_dst != static_cast<int>(this->DTM.size())) {
			DBG::warning(__LINE__, "rgbArrayWidth_dst*rgbArrayHeight_dst != this->DTM.size() windowSize()="+to_string(rgbArrayWidth_dst*rgbArrayHeight_dst)+" this->DTM.size()="+to_string(this->DTM.size()));
			return;
		}
		if (xPos < 0 or xPos >= rgbArrayWidth_dst) {
			DBG::warning(__LINE__, "xPos is out of bounds. xPos = "+to_string(xPos));
			return;
		}
		if (yPos < 0 or yPos >= rgbArrayHeight_dst) {
			DBG::warning(__LINE__, "yPos is out of bounds. yPos="+to_string(yPos));
			return;
		}
		
		float factor = 65536.f/static_cast<float>(maxTerrainElevation);
		posElevation = posElevation*factor + static_cast<float>(this->DTM[xPos+yPos*rgbArrayWidth_dst]);
		findAtElevation=static_cast<float>(findAtElevation)*factor;
		DBG::note(__LINE__, "corrPos terrain height "+to_string((float)this->DTM[xPos+yPos*rgbArrayWidth_dst]/(65536.0/(float)maxTerrainElevation)));
		
		float xPosFloat = static_cast<float>(xPos);
		float yPosFloat = static_cast<float>(yPos);
		float itterationLength = 0.9f;

		// top then bottom
		#pragma omp parallel for
		for (int x = 0; x < rgbArrayWidth_dst; ++x) {
			for (int y = 0; y < rgbArrayHeight_dst; y+=rgbArrayHeight_dst-1) {
				float xDist = static_cast<float>(x)-xPosFloat;
				float yDist = static_cast<float>(y)-yPosFloat;
				float totDist =  sqrtf(xDist*xDist+yDist*yDist);
				float xDirection = itterationLength*(xDist/totDist);
				float yDirection = itterationLength*(yDist/totDist);
				float zSteepness = -99.f;
				float fEnd = totDist/itterationLength;
				for (float f = 1.f; f < fEnd; f+=1.f) {
					int X = static_cast<int>(xPosFloat + f*xDirection);
					int Y = static_cast<int>(yPosFloat + f*yDirection);
					float currentHeight = static_cast<float>(this->DTM[X+Y*rgbArrayWidth_dst]);
					float itterativeHeight = posElevation+zSteepness*(itterationLength*f);
					if (itterativeHeight<currentHeight+findAtElevation) {
						int index = 2 + 3*X + 3*rgbArrayWidth_dst*Y;
						unsigned char prevBlue = rgbArray_dst[index];
						rgbArray_dst[index] = static_cast<unsigned char>(alpha*256.f + (1.f-alpha)*static_cast<float>(prevBlue));
						if (itterativeHeight<currentHeight) {
							zSteepness = (currentHeight-posElevation)/(itterationLength*f);
						}
					}
				}
			}
		}

		// left then right
		#pragma omp parallel for
		for (int y = 0; y < rgbArrayHeight_dst-1; ++y) {
			for (int x = 0; x < rgbArrayWidth_dst; x+=rgbArrayWidth_dst-1.f) {
				float xDist = static_cast<float>(x)-xPosFloat;
				float yDist = static_cast<float>(y)-yPosFloat;
				float totDist =  sqrtf(xDist*xDist+yDist*yDist);
				float xDirection = itterationLength*(xDist/totDist);
				float yDirection = itterationLength*(yDist/totDist);
				float zSteepness = -99.f;
				float fEnd = totDist/itterationLength;
				for (float f = 1.f; f < fEnd; f+=1.f) {
					int X = static_cast<int>(xPosFloat + f*xDirection);
					int Y = static_cast<int>(yPosFloat + f*yDirection);
					float currentHeight = static_cast<float>(this->DTM[X+Y*rgbArrayWidth_dst]);
					float itterativeHeight = posElevation+zSteepness*(itterationLength*f);
					if (itterativeHeight<currentHeight+findAtElevation) {
						int index = 2 + 3*X + 3*rgbArrayWidth_dst*Y;
						unsigned char prevBlue = rgbArray_dst[index];
						rgbArray_dst[index] = static_cast<unsigned char>(alpha*256.f + (1.f-alpha)*static_cast<float>(prevBlue));
						if (itterativeHeight<currentHeight) {
							zSteepness = (currentHeight-posElevation)/(itterationLength*f);
						}
					}
				}
			}
		}
	}
	void terrainAndTreeDataToRGBarray() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		
		if (static_cast<int>(this->rgbArray.size()) != 3 * this->winWidth * this->winHeight) {            
			this->rgbArray.resize(3 * this->winWidth * this->winHeight);
			DBG::note(__LINE__, fmt::format("resizing rgbArray to {} {}", this->winWidth, this->winHeight));
		}
		
		if (this->rgbArray.size() != 3*this->DTM.size() or this->rgbArray.size() != 3*this->DOMminusDTM.size()) {
			DBG::error(__LINE__, fmt::format("rgbArray({}), DTM({}) and DOMminusDTM({}) is not the same size", this->rgbArray.size(), this->DTM.size(), this->DOMminusDTM.size()));
		}

		// Find lowest and highest elevation values in DTM
		auto [minIt, maxIt] = minmax_element(DTM.begin(), DTM.end());
		unsigned short lowest = *minIt;
		unsigned short highest = *maxIt;
		
		unsigned short metersPerPixel = static_cast<unsigned short>(metersWidth/winWidth);
		unsigned short distBetweenLines = 2*metersPerPixel;
		
		DBG::note(__LINE__, fmt::format("lowset={}         highest={}", lowest, highest));
		DBG::note(__LINE__, fmt::format("winWidth={}       winHeight={}", this->winWidth, this->winHeight));
		DBG::note(__LINE__, fmt::format("metersPerPixel={} distBetweenLines={}", metersPerPixel, distBetweenLines));
		printf("mapType=%d\n", this->mapType);
		
		// Update image
		if (this->mapType==0) {
			#pragma omp parallel for collapse(2)
			for (int y = 0; y < this->winHeight; ++y) {
				for (int x = 0; x < this->winWidth; ++x) {
					int index = x + this->winWidth*y;
					unsigned char* red =   &rgbArray[0 + 3*index];
					unsigned char* green = &rgbArray[1 + 3*index];
					unsigned char* blue =  &rgbArray[2 + 3*index];
					
					const float steepness_0to1 = Image::getSteepness_2x2(this->DTM, this->winWidth, this->winHeight, x, y, metersPerPixel);
					const float sunShadow_0to1 = Image::getSunShadow_2x2(this->DTM, this->winWidth, this->winHeight, x, y, 1.1f, 1.0f);
					const float terrainHeight_0to1 = Image::getHeight_0to1(this->DTM, this->winWidth, lowest, highest, x, y);
					const unsigned char heightLine = Image::getHeightLine_2x2(this->DTM, this->winWidth, this->winHeight, x, y, distBetweenLines);

					Image::getCustomColor4(red, green, blue, steepness_0to1, sunShadow_0to1, terrainHeight_0to1, heightLine, this->DOMminusDTM[index]);
				}
			}
		}
		else if (this->mapType==1) {
			#pragma omp parallel for collapse(2)
			for (int y = 0; y < this->winHeight; ++y) {
				for (int x = 0; x < this->winWidth; ++x) {
					int index = x + this->winWidth*y;
					unsigned char* red =   &rgbArray[0 + 3*index];
					unsigned char* green = &rgbArray[1 + 3*index];
					unsigned char* blue =  &rgbArray[2 + 3*index];
					
					const float steepness_0to1 = Image::getSteepness_2x2(this->DTM, this->winWidth, this->winHeight, x, y, metersPerPixel);
					const float sunShadow_0to1 = Image::getSunShadow_2x2(this->DTM, this->winWidth, this->winHeight, x, y, 1.1f, 1.0f);
					const float terrainHeight_0to1 = Image::getHeight_0to1(this->DTM, this->winWidth, lowest, highest, x, y);

					Image::getCustomColor2(red, green, blue, steepness_0to1, sunShadow_0to1, terrainHeight_0to1, this->DOMminusDTM[index]);
				}
			}
		}
		else {
			DBG::warning(__LINE__, fmt::format("mapType not defined. mapType={}", this->mapType));
			this->mapType = 1;
			#pragma omp parallel for collapse(2)
			for (int y = 0; y < this->winHeight; ++y) {
				for (int x = 0; x < this->winWidth; ++x) {
					int index = x + this->winWidth*y;
					unsigned char* red =   &rgbArray[0 + 3*index];
					unsigned char* green = &rgbArray[1 + 3*index];
					unsigned char* blue =  &rgbArray[2 + 3*index];
					
					const float steepness_0to1 = Image::getSteepness_2x2(this->DTM, this->winWidth, this->winHeight, x, y, metersPerPixel);
					const float sunShadow_0to1 = Image::getSunShadow_2x2(this->DTM, this->winWidth, this->winHeight, x, y, 1.1f, 1.0f);
					const float terrainHeight_0to1 = Image::getHeight_0to1(this->DTM, this->winWidth, lowest, highest, x, y);
					const unsigned char heightLine = Image::getHeightLine_2x2(this->DTM, this->winWidth, this->winHeight, x, y, distBetweenLines);

					Image::getCustomColor4(red, green, blue, steepness_0to1, sunShadow_0to1, terrainHeight_0to1, heightLine, this->DOMminusDTM[index]);
				}
			}
		}
	}
	void writeToRGBarrayPtr(unsigned char* rgbArray_dst, int rgbArrayWidth_dst, int rgbArrayHeight_dst, int mouseX, int mouseY) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		
		if (this->winWidth != rgbArrayWidth_dst or this->winHeight != rgbArrayHeight_dst) {
			DBG::error(__LINE__, "cont write to image of other size");
		}
		
		for (int i = 0; i < 3*rgbArrayWidth_dst*rgbArrayHeight_dst; i++) {
			rgbArray_dst[i]=this->rgbArray[i];
		}
		
		this->blueAreaToRGBarrayPtr(rgbArray_dst, rgbArrayWidth_dst, rgbArrayHeight_dst, mouseX, mouseY);
	}
    void updateSize(int newWinWidth, int newWinHeight) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (this->winWidth == newWinWidth and this->winHeight == newWinHeight) {
			DBG::note(__LINE__, "you tried to resize map with the same size");
			return;
		}
		double metersPerPixelX = static_cast<double>(this->metersWidth)/static_cast<double>(this->winWidth);
		DBG::note(__LINE__, fmt::format("prevWinSize=({}, {}) prevMeterSize({}, {})", winWidth, winHeight, metersWidth, metersHeight));
		this->winWidth = newWinWidth;
		this->winHeight = newWinHeight;
		
		this->metersWidth = static_cast<int>(static_cast<double>(winWidth)*metersPerPixelX);
		this->metersHeight = this->metersWidth*newWinHeight/newWinWidth;
		DBG::note(__LINE__, fmt::format("currWinSize=({}, {}) currMeterSize({}, {})", winWidth, winHeight, metersWidth, metersHeight));
		
		this->updateSubMaps();
		this->updateMap();
		this->terrainAndTreeDataToRGBarray();
	}
};