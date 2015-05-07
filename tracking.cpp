
#include <iostream>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include "tracking.h"

#include <fstream>

#include <cmath>
#include <list>
#include <stack>
#include <set>

#include <zmq.h>
#include <zmq_utils.h>

#define and &&

#define NODEGROUP 4
using namespace cv;
using namespace std;

static int th_lower = 10000;
static int th_upper = 200000;

//static int th_lower = 0;
//static int th_upper = 200000;

int ang_dist = 20;
int group_min_distance = 0;
int group_max_distance = 50;
float group_min_radius = 1;
float group_max_radius = 70;

void threashold_ir(Mat * ir, int lower, int upper, Mat* th)
{
	inRange(*ir, Scalar(lower), Scalar(upper), *th);
}

bool check_angles(int a, int b)
{
	int sum = 0;
	if (a < 0 && b > 0)
		sum = abs(a-b);
	else if (a > 0 && b < 0)
		sum = abs(b-a);
	else
		return false;

	if (170 < sum && sum < 190)
		return true;
	return false;
}

int mydistance(Point2f a, Point2f b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void detect_cfs(Mat * ir, Mat * depth, Mat * th_display, cf_instance* out) {


    /* Zero the struct */
    memset(out, 0, sizeof(cf_instance));

    Mat th;
    threashold_ir(ir, th_lower, th_upper, &th);

	int dilation_size = 1;
	Mat element = getStructuringElement(MORPH_ELLIPSE,
		Size(dilation_size + 1, dilation_size + 1),
		Point(dilation_size, dilation_size));

	erode(th, th, element);
	dilate(th, th, element);
	//erode(th, th, element);
	th_display->setTo(Scalar(0, 0, 0), th);
	imshow("TH", th);
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

	findContours(th, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Create "drawable" threashold image

    //putText(th_display, "Test", cvPoint(30,30),
    //	FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(100,10,250), 1, CV_AA);

    // Create a list of all the contours with center and readious
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    //vector<vector<int> > groups;
    vector<bool> grouped_nodes(contours.size());

    char id_txt_buffer [50];
    for( int i = 0; i < contours.size(); i++ )
    {
        //approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( contours[i], center[i], radius[i]);
		circle(*th_display, center[i], radius[i], Scalar(255, 0, 255), 1);
        // Set nodes not grouped
        grouped_nodes[i] = false;
        sprintf (id_txt_buffer, "%d", i);
        putText(*th_display, id_txt_buffer, center[i],
        	FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(100,10,250), 1, CV_AA);
    }

    list<node_group> groups;

    bool sent = false;
    static float x=0, y=0, z=0, angle=0;
		for( int i = 0; i < contours.size(); i++ )
		{
			// First check if this nodes already belongs to a group
			if (!grouped_nodes[i])
			{
				node_group new_group;
				new_group.id = groups.size() + 1;
				cout << "Found new group: " << new_group.id << endl;

				// Now travers all nodes
				list<int> nodes;
				nodes.push_back(i);
				while (!nodes.empty())
				{
					// Pop current node
					int node = nodes.back();
					nodes.pop_back();
					// Mark node as found and add it to the group
					grouped_nodes[node] = true;
					new_group.nodes.push_back(node);
					cout << "Now processing node " << node << ": radius=" << radius[node] << endl;
					// Look at adjacent nodes and add them to the stack
					for (int j = 0; j < contours.size(); j++)
					{
						// Don't check outselves or nodes already marked for checking
						if (j != i and j != node and find(nodes.begin(), nodes.end(), j) == nodes.end() and group_min_radius <= radius[node] and radius[node] <= group_max_radius)
						{
							int distance = sqrt(pow(abs(center[node].x - center[j].x), 2) + pow(abs(center[node].y - center[j].y), 2));
							cout << "Distance from " << node << " to " << j << ": " << distance << "(" << group_min_distance << "-"<<group_max_distance << ")" << endl;
							if (group_min_distance <= distance and distance <= group_max_distance and !grouped_nodes[j])
							{
								cout << "Found adjacent node to " << node << ": " << j << endl;
								nodes.push_back(j);
							}
						}
					}
				}
				cout << "Finished searching group " << new_group.id << ", found " << new_group.nodes.size() << " nodes: ";
				for (list<int>::iterator it = new_group.nodes.begin(); it != new_group.nodes.end(); it++)
					cout << *it << " ";
				cout << endl;
				// Finally add the group to the list
				if (new_group.nodes.size() == NODEGROUP)
				{
					Point2f c[NODEGROUP];
					int pi[NODEGROUP];
					int angles[NODEGROUP][NODEGROUP];
					//vector<int> points;
					int yai = 0;
					set<int> hackaround;
					for (list<int>::iterator it = new_group.nodes.begin(); it != new_group.nodes.end(); it++)
					{
						c[yai]= center[*it];
						pi[yai] = *it;
						hackaround.insert(yai);
						yai += 1;
					}
					// For each node look at the angles to all the others
					for (int a = 0; a < NODEGROUP; a++)
					{
						for (int b = 0; b < NODEGROUP; b++)
						{
							if (a != b)
							{
								angles[a][b] = int(atan2(c[a].y-c[b].y, c[a].x-c[b].x)*180/PI);
								//int angle = int(atan((center[points[b]].y-center[points[a]].y)/(center[points[b]].x-center[points[a]].x))*180/3.14);
								cout << pi[a] << " to " << pi[b] << ": " << angles[a][b] << endl;
							}
						}
					}
					bool actually_found_group = false;
					for (int a = 0; a < NODEGROUP; a++)
					{
						for (int b = 0; b < NODEGROUP; b++)
						{
							for (int c = 0; c < NODEGROUP; c++)
							{
								/*int test = (abs(angles[a][b]) + abs(angles[a][c]));
								//cout << a << ": " << b << "->" << c << "=" << test << endl;
								if ( 170 <= test and test <= 190 and a != b and a != c and b != c)
								{
									cout << a << ", " << b << ", " << c << " is the base (" << test << ")" << endl;
								}*/
								if (check_angles(angles[a][b], angles[a][c]) and hackaround.size() > 2)
								{
									cout << "Base is " << pi[a] << "!!!" << endl;
									circle(*th_display, center[pi[a]], radius[pi[a]], Scalar(0,255,0), 2);
									hackaround.erase(a);
									hackaround.erase(b);
									hackaround.erase(c);
									if (hackaround.size() > 0) {
										int tip = *(hackaround.begin());
										cout << "Tip is " << tip << endl;
										circle(*th_display, center[pi[tip]], radius[pi[tip]], Scalar(255, 0, 0), 2);
										line(*th_display, center[pi[tip]], center[pi[a]], Scalar(0, 0, 255), 2);
										Point2f pt;
										Point2f depth_node;
										cout << "Tip angle: " << angles[a][tip] << endl;
										cout << "Tip angle: " << angles[a][tip] + 180 << endl;
										float mirror = angles[a][tip] + 180;
										double distance = norm(center[pi[tip]] - center[pi[a]]);

										depth_node.x = center[pi[a]].x - cos(mirror*PI / 180)*distance;
										depth_node.y = center[pi[a]].y - sin(mirror*PI / 180)*distance;
										pt.x = center[pi[a]].x - cos(angles[a][tip] * PI / 180) * 100;
										pt.y = center[pi[a]].y - sin(angles[a][tip] * PI / 180) * 100;
										line(*th_display, center[pi[a]], pt, Scalar(0, 0, 0), 2);

										//circle(*th_display, depth_node, radius[pi[tip]], Scalar(150, 150, 150), 2);
										//line(th_display, center[pi[a]], depth_node, Scalar(100,100,100), 2);

										/*
										circle(*th_display, depth_node, radius[pi[tip]], Scalar(150,150,150), 2);
										void * bsh = (static_cast<void*>(depth->data));
										float * data = (static_cast<float*>(bsh));
										data += int(depth_node.y) * 512 + int(depth_node.x);
										cout << "Depth " << *data << "mm (" << (*data)/10/100 << "m)"<< endl;

										sprintf (id_txt_buffer, "%.2f", (*data)/10/100);
										putText(*th_display, id_txt_buffer, depth_node,
										FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(100,10,250), 1, CV_AA);*/

										out->x = center[pi[a]].x;
										out->y = center[pi[a]].y;
										out->angle = (angles[a][tip] + 45 + 360) % 360;
										out->x_d = depth_node.x;
										out->y_d = depth_node.y;
										out->found = true;

										actually_found_group = true;
									}
								}
							}
						}
					}
					if (actually_found_group) {
						cout << " -> OK" << endl;
						groups.push_back(new_group);
					}

				} else {
					cout << " -> NOT OK, discarding group!" << endl;
				}
			}
		}
}

