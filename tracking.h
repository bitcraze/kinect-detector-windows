
#include <opencv2/opencv.hpp>
#include <list>

#ifndef _TRACKING_H_
#define _TRACKING_H_

using namespace cv;
using namespace std;

struct node_group {
	int id;
    list<int> nodes;
};

#define PI 3.14159265

typedef struct {
	float x;
	float y;
	float x_d;
	float y_d;
	float z;
	float angle;
	bool found;
} cf_instance;

void detect_cfs(Mat * ir, Mat * depth, Mat * th_display, cf_instance* out);

/* Only for libfreenect2 */
void get_depth_at_xy(void * data, cf_instance * out);

#endif /*_TRACKING_H_*/
