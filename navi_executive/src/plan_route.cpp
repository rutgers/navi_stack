/*
 * 
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <gps_common/conversions.h>
#include <vector>

using namespace std;
typedef struct latlong {
	float lat;
	float lon;
} latlong;


istream& operator>> (istream& is,  latlong& coord){
	is >> coord.lat;
	is >> coord.lon;
}

ostream& operator<< (ostream& s,  latlong& coord){
	s << " ";
	s << coord.lat; 
	s << " "; 
	s << coord.lon;
	s << " ";
}

int main(int argc, char** argv){
	
	fstream fcoords ("test.coords", fstream::in );
	
	std::vector<latlong> coords;

	while (! fcoords.eof()){
		latlong coord;
		fcoords >> coord;
		coords.push_back(coord);
	}
	cout << "The robot starts at" << coords[0] << '\n';
	
	
	
}
