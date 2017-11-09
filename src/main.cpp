#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
		{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

		}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
		{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

		}

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	double ref_vel = 0.3; // start at mph
	int chosen_state;
	int current_lane = -1;

	h.onMessage([&max_s, &current_lane, &chosen_state, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// Define relevant parameters
		double desired_vel = 50.0; // mph
		int N_waypoints = 3; // Number of future waypoints to use for the spline
		double waypoint_spacing = 50.0; // m
		int N_path = 50;
		int N_planner = 120;

		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Initialize current_lane variable with the current position
					if (current_lane<0)
						current_lane = int(car_d)/4;

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// Size of previous path that hasn't yet been executed by the simulator
					int prev_size = previous_path_x.size();

					// Use the previous path returned by the simulator to generate the first point which will be the same for all trajectories
					double ref_x_prev,ref_x;
					double ref_y_prev,ref_y;
					double ref_yaw_prev,ref_yaw;

					// If previous path is nearly empty, use current position of the car as reference point
					if (prev_size<2)
					{
						ref_yaw = deg2rad(car_yaw);

						ref_x_prev = car_x-cos(ref_yaw);
						ref_x = car_x;

						ref_y_prev = car_y-sin(ref_yaw);
						ref_y = car_y;
					}
					// Else, we use the previous path as starting point for our new trajectory
					else
					{
						ref_x_prev = previous_path_x[prev_size-2];
						ref_y_prev = previous_path_y[prev_size-2];

						ref_x = previous_path_x[prev_size-1];
						ref_y = previous_path_y[prev_size-1];

						ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
					}

					// Behavior Planning: generate different trajectories and calculate a cost for each
					// First, generate trajectories for each of the possible states:
					// 1) KL (Keep Lane)
					// 2) LCL (Lange Change Left)
					// 3) LCR (Lange Change Right)

					enum state_name { KL, LCL, LCR};
					struct State {
						state_name name;
						vector<double> next_x_vals;
						vector<double> next_y_vals;
						vector<double> next_s_vals;
						vector<double> next_d_vals;
						int lane;
						double vel;
						double cost = 0;
					};

					// Always create KL trajectory
					vector<State> states;
					State state;
					state.lane = current_lane;
					state.vel = ref_vel;
					state.name = KL;
					states.push_back(state);

					// Create LCL and LCR trajectories only when the car is in the center of its planned lane
					// This way, a new lane change is not initiated before the previous lane change is carried out.
					if ((car_d < 2+4*current_lane+0.5) && (car_d > 2+4*current_lane-0.5))
					{
						// Only do LCL if we are not in the leftmost lane
						if (current_lane>=1)
						{
							state.name = LCL;
							state.lane = current_lane-1;
							states.push_back(state);
						}
						// Only do LCR if we are not in the rightmost lane
						if (current_lane<=1)
						{
							state.name = LCR;
							state.lane = current_lane+1;
							states.push_back(state);
						}
					}

					// Add duplicates of feasible states with increased and decreased velocities
					int states_old_size = states.size();
					for (size_t s=0;s<states_old_size;s++)
					{
						State state_tmp = states[s];
						state_tmp.vel = states[s].vel+0.224;
						if ((state_tmp.vel <= desired_vel))
							states.push_back(state_tmp);
						state_tmp.vel = states[s].vel-0.224;
						if (state_tmp.vel >= 0)
							states.push_back(state_tmp);
					}

					// Generate trajectories for each of the possible states.
					// For each state, we generate two overlapping trajectories:
					// Trajectory 1) Short horizon. It is used for actually controlling the vehicle
					// Trajectory 2) Long horizon. It is used for planning into the future and for comparing the different states
					for (size_t s=0;s<states.size();s++)
					{
						// We first generate trajectory 1 (used for controlling the vehicle)
						// Generate a list of widely spaced waypoints along the trajectory (in global x and y coordinates)
						vector<double> ptsx;
						vector<double> ptsy;

						// First, we add the last point we sent to the simulator, by using the previous path returned by the simulator
						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);

						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);

						// We then add waypoints further ahead spaced 50m apart
						for (size_t i=0;i<N_waypoints;i++)
						{
							vector<double> waypoint = getXY(car_s+waypoint_spacing*(i+1),2+4*states[s].lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);

							ptsx.push_back(waypoint[0]);
							ptsy.push_back(waypoint[1]);
						}

						// Transform from global map coordinates to local vehicle coordinates
						for (size_t i=0;i<ptsx.size();i++)
						{
							// Translation
							double shift_x = ptsx[i]-ref_x;
							double shift_y = ptsy[i]-ref_y;

							// Rotation
							ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
							ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
						}

						// We then want to sample along these widely spaced waypoints.
						// This is done by fitting a spline and sampling that function.

						// Create a spline
						tk::spline s1;

						// Add the waypoints to the spline
						s1.set_points(ptsx,ptsy);

						// Define list of query points on the spline that we want to send to our simulator
						// Start by adding all previous points from the simulator (ensure smooth transition)
						for (size_t i=0;i<prev_size;i++)
						{
							states[s].next_x_vals.push_back(previous_path_x[i]);
							states[s].next_y_vals.push_back(previous_path_y[i]);
						}

						// Make sure to add evenly spaced points along the spline.
						// For this, we use a linear approximation from the first to the second waypoint.
						double target_x = waypoint_spacing;
						double target_y = s1(target_x);
						double target_dist = sqrt((target_x*target_x)+target_y*target_y);
						double x_add_on = 0;
						for (size_t i=0;i<N_path-prev_size;i++)
						{
							double N = target_dist/(0.02*states[s].vel/2.24); // Compute spacing between points, and convert from mph to m/s
							double x_point = x_add_on+target_x/N;
							double y_point = s1(x_point);

							x_add_on = x_point;

							// Transform back from local vehicle coordinates to global map coordinates
							double x_ref = x_point;
							double y_ref = y_point;

							// Rotation
							x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
							y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

							// Translation
							x_point += ref_x;
							y_point += ref_y;

							// Add point to path list
							states[s].next_x_vals.push_back(x_point);
							states[s].next_y_vals.push_back(y_point);
						}

						// We then generate trajectory 2 (used for planning)

						// Create a spline
						tk::spline s2;

						// Generate a list of widely spaced waypoints along the trajectory (this time in Frenet coordinates)
						vector<double> ptss;
						vector<double> ptsd;

						// Add current position of the car
						ptss.push_back(car_s);
						ptsd.push_back(car_d);

						// Add waypoints further ahead spaced 50m apart
						for (size_t i=0;i<N_waypoints;i++)
						{
							ptss.push_back(car_s+waypoint_spacing*(i+1));
							ptsd.push_back(2+4*states[s].lane);
						}

						// Add the waypoints to the spline
						s2.set_points(ptss,ptsd);


						// Define list of query points on the spline that make up the planned path used for evaluating states
						for (size_t i=0;i<N_planner;i++)
						{
							double next_s = car_s+((double)i)*(0.025*states[s].vel/2.24);
							double next_d = s2(next_s);
							states[s].next_s_vals.push_back(next_s);
							states[s].next_d_vals.push_back(next_d);
						}
					}

					// Before evaluating the generated states, we augment the sensor_fusion list.
					// As other cars are only modeled as points, we extend the representation and model each car with 2 points.
					// One point 3 meters in front of car center, and one point 2 meters behind the car center.
					// This way, we effectively avoid hitting the front and rear ends of other cars when changing lanes.
					int sensor_fusion_old_size = sensor_fusion.size();
					for (size_t i=0;i<sensor_fusion_old_size;i++)
					{
						double check_car_s = sensor_fusion[i][5];
						sensor_fusion[i][5] = check_car_s+3;
						auto new_car = sensor_fusion[i];
						new_car[5] = check_car_s-2;
						sensor_fusion.push_back(new_car);
					}

					// Handle wrap-around of sensor_fusion s-values when our car approaches the end (loop-closure) of the track.
					if (car_s > max_s-500)
					{
						for (size_t i=0;i<sensor_fusion.size();i++)
						{
							double check_car_s = sensor_fusion[i][5];

							if (check_car_s<500)
								sensor_fusion[i][5] = check_car_s+max_s;
						}
					}
					std::cout<<std::endl;

					// Calculate costs for planned trajectories
					for (size_t s=0;s<states.size();s++)
					{
						// Print for debug
						std::cout<<"State: "<<s<<" (lane "<<states[s].lane<<")"<<std::endl;

						// Calculate maximum safe travel distance for the given state

						// If there were no cars on the road, the safe distance would be the length of the s trajectory
						int max_safe_dist_ind = states[s].next_s_vals.size()-1;
						double max_safe_dist = states[s].next_s_vals[max_safe_dist_ind]-states[s].next_s_vals[0];
						// Print for debug
						std::cout<<"  without cars: max_safe_dist="<<max_safe_dist<<", max_safe_dist_ind="<<max_safe_dist_ind<<std::endl;

						// Loop through the sensor_fusion list to find potential collisions with other cars
						for (size_t i=0;i<sensor_fusion.size();i++)
						{
							float d = sensor_fusion[i][6];

							// Only look at cars in the desired lane
							if (d < (2+4*states[s].lane+2) && d > (2+4*states[s].lane-2))
							{
								// Extract current s coordinate and velocity of the other car
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx*vx+vy*vy);
								double check_car_s = sensor_fusion[i][5];

								// Search along the evaluated trajectory of our car until there is a collision.
								// The longest distance before a collision is the safe travel distance for this trajectory.
								double prev_dist;
								for (size_t t=0;t<states[s].next_s_vals.size();t++)
								{
									// Predict future s coordinate of the other car
									double check_car_s_next = check_car_s+(double)t*0.02*check_speed;

									// Calculate the signed distance between the other car and our car for time step t
									double dist = check_car_s_next-states[s].next_s_vals[t];

									// Calculate the traversed distance of our car so far along the evaluated trajectory
									double safe_travel_dist = states[s].next_s_vals[t]-states[s].next_s_vals[0];

									// A collision can happen with a car in front of us or a car approaching from behind.
									// We therefore look for when the calculated distance changes sign.
									// Further, we are only interested in collisions happening at closer distances than what we have seen so far.
									if ((t>0) && (prev_dist*dist<0) && (safe_travel_dist<max_safe_dist))
									{
										// Update maximum safe travel distance for the given state
										max_safe_dist = states[s].next_s_vals[t-1]-states[s].next_s_vals[0];
										max_safe_dist_ind = t-1;
									}
									prev_dist = dist; // Store previous distance (used for sign change detection)
								}
							}
						}

						// Print for debug
						std::cout<<"  with cars   : max_safe_dist="<<max_safe_dist<<", max_safe_dist_ind="<<max_safe_dist_ind<<std::endl;

						// Calculate a cost that we want to minimize.
						// The cost is simply the inverse of the maximum travel distance for the evaluated trajectory.
						// A long trajectory (which is desired) will then have a small cost, and vice versa.
						states[s].cost = 1/(max_safe_dist+0.001f);

						// Print for debug
						std::cout<<"  cost: "<<states[s].cost<<", velocity: "<<states[s].vel<<std::endl;
					}

					// Choose trajectory with lowest cost
					double min_cost = std::numeric_limits<double>::max();
					for (size_t s=0;s<states.size();s++)
					{
						if (states[s].cost < min_cost)
						{
							min_cost = states[s].cost;
							chosen_state = s;
						}
					}
					state = states[chosen_state];

					// Print for debug
					std::cout<<"Chosen state: "<<chosen_state<<" (lane "<<state.lane<<")"<<std::endl;
					std::cout<<std::endl;

					// Update current lane and velocity
					current_lane = state.lane; // Actually not the current lane, but the planned lane
					ref_vel = state.vel;

					// Send the trajectory used for controlling the vehicle of the chosen state
					msgJson["next_x"] = state.next_x_vals;
					msgJson["next_y"] = state.next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
