/*
 * 
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <gps_common/conversions.h>
#include <vector>
#include <string>
#include <cmath>

using namespace std;
typedef struct latlong {
	double lat;
	double lon;
} latlong;

typedef struct UTM {
	double northing;
	double easting;
	string zone;
} UTM;

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


ostream& operator<< (ostream& s,  UTM& coord){
	s << " ";
	s << coord.easting;
	s << " ";
	s << coord.northing;
	s << " ";
}


void planRoute(std::vector<UTM>& waypoints, std::vector<UTM>& plan, std::vector<int>& wp){

	std::vector<bool> used;
	used.resize(waypoints.size(), false);
	plan.clear();
	wp.clear();
	wp.push_back(0);
	plan.push_back(waypoints[0]);

	used[0] = true; //the first one is the start location

	int num_used =1;
	int c_point =0;

	while (num_used!=used.size()){

		int best_point;
		double dist =9909999;
		for(int i=0; i<waypoints.size(); i++){
			if (!used[i]){
				double d = fabs(waypoints[i].easting - waypoints[c_point].easting)+ fabs(waypoints[i].northing-waypoints[c_point].northing);
				if (d< dist){
					best_point = i;
					dist = d;
				}
			}
		}

		plan.push_back(waypoints[best_point]);
		used[best_point] = true;
		wp.push_back(best_point);
		num_used++;
	}

}

void cvtLLToUTM(std::vector<latlong>& ll, std::vector<UTM>& utm){
	utm.resize(ll.size());

	for (int i =0; i < ll.size(); i++){
		double northing, easting;
		std::string zone;
		gps_common::LLtoUTM(ll[i].lat, ll[i].lon, northing, easting, zone);
		utm[i].northing = northing;
		utm[i].easting = easting;
		utm[i].zone = zone;
	}


}
int main(int argc, char** argv){
	
	cout.precision(10);

	string inFile = argv[1];
	fstream fcoords (inFile.c_str(), fstream::in );
	
	fcoords.precision(10);

	std::vector<latlong> coords;

	while (! fcoords.eof()){
		latlong coord;
		char temp[100];
		fcoords.getline(temp,100);
		stringstream ss(temp);
		ss >>coord;
		cout << coord << endl;
		coords.push_back(coord);
	}
	

	std::vector<UTM> coords_utm;
	cvtLLToUTM(coords,coords_utm);

	std::vector<UTM> route;
	std::vector<int> wps;

	planRoute(coords_utm, route,wps);

	cout<< "The transformed UTM coordinates are\n";
	for (int i=0; i<coords_utm.size(); i++){
			cout <<  coords_utm[i] << '\n';
		}


	fstream fout_coords (( string("routed_")+inFile).c_str(), fstream::out );
	fout_coords.precision(8);
	for (int i=0; i<route.size(); i++){
			cout <<  route[i] << coords[wps[i]] << '\n';
		}
}
