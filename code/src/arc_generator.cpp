#include <boost/geometry.hpp>
#include "drawing.h"
#include <vector>
#include <set>
#include <map>
#include <utility>
#include <Eigen/Dense>
#include <deque>

namespace veiv{
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3;
typedef Drawing::Point2d Point2;


// Use SVD to figure out the pricipal direction with small variance
void PCA_2D(const std::vector<Point2> &pts, Point2 &centroid, Point2 &dir)
{
	typedef Eigen::Matrix<double, Eigen::Dynamic, 2> MatrixX2d;

	int m = pts.size();
	MatrixX2d mat(m, 2);
	for(int i = 0; i < m; i++){
		mat(i, 0) = pts[i].x();
		mat(i, 1) = pts[i].y();
	}

	// Compute centroid
	Eigen::Matrix<double, 1, 2> mean_val = mat.colwise().mean();
	centroid.x() = mean_val(0,0);
	centroid.y() = mean_val(0,1);

	for(int i = 0; i < m; i++){
		mat.row(i) -= mean_val;
	}

	Eigen::JacobiSVD<MatrixX2d> jsvd(mat, Eigen::ComputeFullV);
	Eigen::Vector2d vec = jsvd.matrixV().col(1);
	dir.x() = vec(0);
	dir.y() = vec(1);
}


void sort_intersection_info(const std::vector< std::vector<Point2> > &curve_points, const std::vector< Point2 > &int_points,
		const std::vector< int > &source_curve_idx, const std::vector<int> &source_segment_idx,
		const std::vector< int > &target_curve_idx, const std::vector<int> &target_segment_idx,
		std::vector< std::vector<Point2> > &curve_intersection_points, std::vector< std::vector<int> > &curve_interrsection_segment_idx)
{
	std::vector< std::set<std::map<int,std::vector<Point2> > > > intersection_point_info(curve_points.size());
	for(int i = 0; i < static_cast<int>(int_points.size()); i++){
		intersection_point_info[source_curve_idx[i]][source_segment_idx[i]].push_back(int_points[i]);
		intersection_point_info[target_curve_idx[i]][target_segment_idx[i]].push_back(int_points[i]);
	}

	curve_intersection_points.resize(intersection_point_info.size());
	curve_interrsection_segment_idx.resize(intersection_point_info.size());

	// Now put them into std vectors
	for(int i = 0; i < static_cast<int>(intersection_point_info.size()); i++){
		for(std::map<int, std::vector<Point2> >::iterator iter = intersection_point_info.begin(); iter != intersection_point_info.end(); ++ iter){
			std::vector<Point2> &pts = iter->second;
			if(static_cast<int>(pts.size()) > 1){
				// Sort the points according to their distance to the common segment starting point
				std::set<std::pair<double, Point2> > sorted_pts;
				Point2 start_pt = curve_points[i][iter->first];
				for(int k = 0; k < static_cast<int>(pts.size()); k++){
					double dist = boost::geometry::distance(start_pt, pts[k]);
					sorted_pts.insert(std::make_pair(dist, pts[k]));
				}
				for(std::set<std::pair<double, Point2>>::iterator iter2 = sorted_pts.begin(); iter2 != sorted_pts.end(); ++ iter2){
					curve_intersection_points[i].push_back(iter2->second);
					curve_interrsection_segment_idx[i].push_back(iter->first);
				}
			}
			else{
				curve_intersection_points[i].push_back(iter->second);
				curve_interrsection_segment_idx[i].push_back(iter->first);
			}
		}
	}
}

void raise_curves(const Point2 &int_point, const std::vector<Point2> &lower_curve, const std::vector<Point2> &upper_curve, int lower_curve_idx, int upper_curve_idx)
{
	// Find out a sequence of segments from the lower curve, and use its principal direction to figure out a proper lifting of the upper curve
	std::deque<Point2> lower_vtx_seq;
	lower_vtx_seq.push_back(int_point);

	for(int i = 0; i < )
	for(int i = 0; i < )
}


void raise_curves(const std::vector< std::vector<Point2> > &curve_points, const std::vector< std::vector<Point2> > &curve_intersection_points,
		const std::vector< std::vector<int> > &curve_interrsection_segment_idx, double radius, std::vector<Point3> &new_curve_points)
{
	// For each interseciton piece, make sure the
}

void generate_arc(const std::vector< std::vector<Point2> > &curve_points, const std::vector< Point2 > &int_points,
		const std::vector< int > &source_curve_idx, const std::vector<int> &source_segment_idx,
		const std::vector< int > &target_curve_idx, const std::vector<int> &target_segment_idx,
		double radius,
		std::vector< std::vector<Point3> > &output_curve_points)
{
	// First sort the intersection points within one curve
	std::vector< std::set<int> > intersection_info;
	for()
	for(int i = 0; i < )

	// enumerate all intersection points within one curve, and raise arcs if necessary


    //
}

}


