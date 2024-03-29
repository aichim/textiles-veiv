#include "arc_generator.h"
#include "utils.h"
#include "drawing.h"


#include <boost/geometry.hpp>
#include <vector>
#include <set>
#include <map>
#include <utility>
#include <Eigen/Dense>
#include <deque>
#include <cmath>
#include <iostream>

namespace veiv{
//typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3;
//typedef Point2d Point2;

#define OVERHANG_ANGLE 50
#define LOWER_RADIUS_RATIO 6
#define UPPER_RADIUS_RATIO 2


void to_eigen_2d(const Point2 &pt, Eigen::Vector2d &eigen_pt){
	eigen_pt(0) = pt.x();
	eigen_pt(1) = pt.y();
}

Point2 from_eigen_2d(const Eigen::Vector2d &eigen_pt){
	return Point2(eigen_pt(0), eigen_pt(1));
}

Point2 interpolate_point2(const Point2 &pt1, const Point2 &pt2, double val1, double val2, double target_val)
{
	double ratio = (target_val - val1) / (val2 - val1);
	return Point2( pt1.x() + ( pt2.x() - pt1.x() ) * ratio, pt1.y() + ( pt2.y() - pt1.y() ) * ratio );
}

Point3 augmented_point2(const Point2 &pt, double z)
{
	return Point3(pt.x(), pt.y(), z);
}

void augmented_point2_to_eigen_vec3(const Point2 &pt, Eigen::Vector3d &eigen_vec)
{
	eigen_vec(0) = pt.x();
	eigen_vec(1) = pt.y();
	eigen_vec(2) = 0.0;
}

// Use SVD to figure out the pricipal direction with small variance
void PCA_2D(const std::deque<Point2> &pts, Point2 &centroid, Point2 &dir)
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
	centroid.x(mean_val(0,0));
	centroid.y(mean_val(0,1));

	for(int i = 0; i < m; i++){
		mat.row(i) -= mean_val;
	}

	std::cout << mat << std::endl;

	Eigen::JacobiSVD<MatrixX2d> jsvd(mat, Eigen::ComputeFullV);
	Eigen::Vector2d vec = jsvd.matrixV().col(1);
	dir.x(vec(0));
	dir.y(vec(1));
}




void sample_lower_curve(const std::vector<Point2> &curve, const Point2 &mid_point, int segment_idx, double target_length, std::deque<Point2> &pts)
{
	pts.push_back(mid_point);
	double half_length = target_length / 2;

	int m = curve.size();
	int idx_increment[] = {-1, 1};
	int start_idx[] = { (segment_idx+1)%m, segment_idx };
	for(int i = 0; i < 2; i++){
		double total_length = 0.0;
		int cur_idx = start_idx[i];
		Point2 cur_pt = mid_point;

		while(true){
			double remaining_length = half_length - total_length;
			int next_idx = cur_idx + idx_increment[i];

			if(next_idx < 0){
				next_idx += m;
			}
			else{
				next_idx = next_idx % m;
			}

			Point2 candidate_pt = curve[next_idx];
			double candidate_length = boost::geometry::distance(candidate_pt, cur_pt);
			if(candidate_length >= remaining_length){
				Eigen::Vector2d diff(candidate_pt.x() - cur_pt.x(), candidate_pt.y() - cur_pt.y());
				diff *= (remaining_length / candidate_length);
				Point2 final_pt( cur_pt.x() + diff(0), cur_pt.y() + diff(1));
				if(i == 0){
					pts.push_front(final_pt);
				}
				else{
					pts.push_back(final_pt);
				}
				break;
			}
			else{
				if(i == 0){
					pts.push_front(candidate_pt);
				}
				else{
					pts.push_back(candidate_pt);
				}

				cur_pt = candidate_pt;
				cur_idx = next_idx;
				total_length += candidate_length;
			}
		}
	}
}

int find_interval(const std::vector<double> &val, double value){
	int m = val.size();
	for(int i = 1; i < m; i++){
		if(val[i] >= value){
			return i-1;
		}
	}

	return 0;
}

bool overlap_sequence(int prev_start_idx, int prev_end_idx, int next_start_idx, int n_vtx){
	int cur_idx = prev_start_idx - 1;
	while(true){
		cur_idx = (cur_idx + 1) % n_vtx;
		if(cur_idx == prev_end_idx){
			return false;
		}

		if(cur_idx == next_start_idx){
			return true;
		}
	}

	return false;
}


void augment_inbetween_sequence(const std::vector<Point2> &curve_seq,
		const std::deque<bool> &covered_vtx, int prev_start_idx, int prev_end_idx, int next_start_idx,
		double radius, std::vector<Point3> &augmented_sequence)
{
	int m = curve_seq.size();
//	if((next_start_idx + 1) % m == prev_end_idx){
//		return;
//	}

//	if(covered_vtx[prev_end_idx]){
//		return;
//	}

	if(overlap_sequence(prev_start_idx, prev_end_idx, next_start_idx, m)){
		return;
	}

	//int prev_end_segment_idx = prev_end_idx - 1;
	//if(prev_end_segment_idx < 0){
	//	prev_end_segment_idx += m;
	//}

	//if(prev_end_segment_idx != next_start_idx){
		int cur_idx = prev_end_idx;
		while(true){
			if(!covered_vtx[cur_idx]){
				augmented_sequence.push_back(augmented_point2(curve_seq[cur_idx], radius));
			}


			if(cur_idx == next_start_idx){
				return;
			}

			cur_idx = (cur_idx + 1) % m;
		}
	//}
}



void augment_inbetween_sequence(const std::vector<Point2> &curve_seq,
		const std::deque<bool> &covered_vtx, int prev_end_idx, int next_start_idx,
		double radius, std::vector<Point3> &augmented_sequence)
{
	int m = curve_seq.size();

	int cur_idx = prev_end_idx;
	while(true){
		if(!covered_vtx[cur_idx]){
			augmented_sequence.push_back(augmented_point2(curve_seq[cur_idx], radius));
		}


		if(cur_idx == next_start_idx){
			return;
		}

		cur_idx = (cur_idx + 1) % m;
	}
}


void augment_whole_sequence(const std::vector<Point2> &curve_seq,
		double radius, std::vector<Point3> &augmented_sequence)
{
	int m = curve_seq.size();
	for(int i = 0; i < m; i++){
		augmented_sequence.push_back(augmented_point2(curve_seq[i], radius));
	}
}

void raise_single_sequence(const std::deque<Point2> &curve_seq, double radius, std::vector<Point3> &lifted_sequence)
{
	std::vector<double> accum_length;
	int m = curve_seq.size();
	accum_length.push_back(0);
	for(int i = 0; i < m - 1; i++){
		double dist = boost::geometry::distance(curve_seq[i], curve_seq[i+1]);
		accum_length.push_back(accum_length.back() + dist);
	}

	double flat_length = radius * UPPER_RADIUS_RATIO ;
	double overhang_ratio = std::tan( M_PI * OVERHANG_ANGLE / 180);
	double slope_length = (accum_length.back() - flat_length) * 0.5;
	double end_slope_length = flat_length + slope_length;
	double min_h = radius * 2;
	double max_h = min_h + slope_length * overhang_ratio;
	//double mid_length = accum_length.back() * 0.5;

	//int mid_val_idx = find_interval(accum_length, mid_length),
	int slope_length_idx = find_interval(accum_length, slope_length),
			end_slope_length_idx = find_interval(accum_length, end_slope_length);

	Point2 slope_length_pt = interpolate_point2(curve_seq[slope_length_idx], curve_seq[slope_length_idx+1],
			accum_length[slope_length_idx], accum_length[slope_length_idx+1], slope_length);

	Point2 end_slope_length_pt = interpolate_point2(curve_seq[end_slope_length_idx], curve_seq[end_slope_length_idx+1],
			accum_length[end_slope_length_idx], accum_length[end_slope_length_idx+1], end_slope_length);


	lifted_sequence.clear();
	lifted_sequence.push_back(augmented_point2(curve_seq.front(), radius));

	for(int i = 0; i <= slope_length_idx ; i++){
		lifted_sequence.push_back(augmented_point2(curve_seq[i], min_h + accum_length[i] * overhang_ratio));
	}

	lifted_sequence.push_back(augmented_point2(slope_length_pt, max_h));

	for(int i = slope_length_idx + 1; i <= end_slope_length_idx ; i++){
		lifted_sequence.push_back(augmented_point2(curve_seq[i], max_h));
	}

	lifted_sequence.push_back(augmented_point2(end_slope_length_pt, max_h));

	for(int i = end_slope_length_idx + 1; i < m ; i++){
		lifted_sequence.push_back(augmented_point2(curve_seq[i], min_h + (accum_length.back() - accum_length[i]) * overhang_ratio));
	}

	lifted_sequence.push_back(augmented_point2(curve_seq.back(), radius));
}

void sample_upper_curve(const Point2 &int_pt, const Point2 &principal_dir,
		const std::deque<Point2> &lower_sample, const std::vector<Point2> &upper_curve,
		double radius,
		int upper_segment_idx,
		std::vector<Point3> &lifted_upper_samples,
		int &upper_start_idx, int &upper_end_idx)
{
	double max_dist = -1e10, min_dist = 1e10;
	Eigen::Vector2d dir(principal_dir.x(), principal_dir.y()), int_pt_eigen(int_pt.x(), int_pt.y());
	for(int i = 0; i < static_cast<int>(lower_sample.size()); i++){
		Eigen::Vector2d cur_pt(lower_sample[i].x(), lower_sample[i].y());
		double dist = dir.dot(cur_pt - int_pt_eigen);
		max_dist = std::max(max_dist, dist);
		min_dist = std::min(min_dist, dist);
	}

	double upper_min_dist = min_dist - radius * 5;
	double upper_max_dist = max_dist + radius * 5;

	int m = upper_curve.size();
	upper_start_idx = upper_segment_idx;
	upper_end_idx = (upper_segment_idx + 1) % m;

	std::deque<Point2> upper_2d_sample;
	upper_2d_sample.push_back(int_pt);

	// collect points until they are far away enough from the intersection point
	int idx_increment[] = {-1, 1};
	int start_idx[] = { (upper_segment_idx+1)%m, upper_segment_idx };
	for(int i = 0; i < 2; i++){
		for(int k = 1; k < m-1; k++){
			int cur_idx = start_idx[i] + idx_increment[i] * k;
			if(cur_idx < 0){
				cur_idx += m;
			}
			else{
				cur_idx = cur_idx % m;
			}

			Point2 candidate_pt = upper_curve[cur_idx];
			boost::geometry::subtract_point(candidate_pt, int_pt);
			double cur_dist = boost::geometry::dot_product(candidate_pt, principal_dir);

			bool finish = cur_dist <= upper_min_dist || cur_dist >= upper_max_dist;
			if(finish){
				Point2 cur_end_pt = i == 0 ? upper_2d_sample.front() : upper_2d_sample.back();
				Eigen::Vector2d end_pt_eigen, candidate_pt_eigen, mid_int_pt_eigen, principal_dir_eigen;
				to_eigen_2d(cur_end_pt, end_pt_eigen);
				to_eigen_2d(upper_curve[cur_idx], candidate_pt_eigen);
				to_eigen_2d(int_pt, mid_int_pt_eigen);
				to_eigen_2d(principal_dir, principal_dir_eigen);


				double prev_end_dist = principal_dir_eigen.dot(end_pt_eigen - mid_int_pt_eigen);
				//double next_end_pt_dist = principal_dir_eigen.dot(candidate_pt_eigen - mid_int_pt_eigen);
				double next_end_pt_dist = cur_dist;
				double limit_dist = cur_dist <= upper_min_dist ? upper_min_dist : upper_max_dist;

				Point2 final_end_pt = interpolate_point2(upper_curve[cur_idx], cur_end_pt, next_end_pt_dist, prev_end_dist, limit_dist);
				if(i==0){
					upper_2d_sample.push_front(final_end_pt);
					upper_start_idx = cur_idx;
				}
				else{
					upper_2d_sample.push_back(final_end_pt);
					upper_end_idx = cur_idx;
				}

				break;
			}
			else{
				if(i==0){
					upper_2d_sample.push_front(candidate_pt);
				}
				else{
					upper_2d_sample.push_back(candidate_pt);
				}
			}
		}
	}

	raise_single_sequence(upper_2d_sample, radius, lifted_upper_samples);
}


void raise_curves(const Point2 &int_point, const std::vector<Point2> &lower_curve, const std::vector<Point2> &upper_curve,
		int lower_curve_segment_idx, int upper_curve_segment_idx, double radius, std::vector<Point3> &lifted_upper_samples,
		int &upper_start_idx, int &upper_end_idx)
{
	// Find out a sequence of segments from the lower curve, and use its principal direction to figure out a proper lifting of the upper curve
	std::deque<Point2> lower_vtx_seq;
	sample_lower_curve(lower_curve, int_point, lower_curve_segment_idx, radius * 6, lower_vtx_seq);

	Point2 centroid, dir;
	PCA_2D(lower_vtx_seq, centroid, dir);

	sample_upper_curve(int_point, dir, lower_vtx_seq, upper_curve,
			radius, upper_curve_segment_idx, lifted_upper_samples, upper_start_idx, upper_end_idx);
}


// Three possible statuses in the process of determining raising
enum RaisingStatus{
	RAISING_UNDECIDED,
	RAISING_UP,
	RAISING_DOWN
};


template<typename T1, typename T2>
void initialize_sequence_family(const std::vector< std::vector<T1> > &ref_sequence, std::vector< std::vector<T2> > &output_seq)
{
	int m = ref_sequence.size();
	output_seq.clear();
	for(int i = 0; i < m; i++){
		output_seq.push_back( std::vector<T2>(ref_sequence[i].size()) );
	}
}

template<typename T1, typename T2>
void initialize_sequence_family(const std::vector< std::vector<T1> > &ref_sequence, const T2 &init_value, std::vector< std::vector<T2> > &output_seq)
{
	int m = ref_sequence.size();
	output_seq.clear();
	for(int i = 0; i < m; i++){
		output_seq.push_back( std::vector<T2>(ref_sequence[i].size(), init_value) );
	}
}

void process_intersection_pair(const std::vector< std::vector<Point2> > &curve_points,
		const std::vector< std::vector<Point2> > &curve_intersection_points,
		const std::vector< std::vector<int> > &curve_intersection_segment_idx,
		const std::vector< std::vector<int> > &intesecting_curve_idx,
		int current_curve_idx,
		int current_curve_intpt_idx, bool current_curve_is_up, double radius,
		std::vector< std::vector<RaisingStatus> > &raising_status,
		std::vector< std::vector<std::pair<int, int> > > &inserted_seq_end_idx,
		std::vector< std::vector< std::vector<Point3> > > &inserted_seq)
{

	Point2 intersection_point = curve_intersection_points[current_curve_idx][current_curve_intpt_idx];
	int neighbor_curve_idx = intesecting_curve_idx[current_curve_idx][current_curve_intpt_idx];

	// First find out the segment index for the neighboring curve
	double min_dist = 1e16;
	int m = curve_intersection_points[neighbor_curve_idx].size();
	int neighbor_curve_intpt_idx = -1;
	for(int i = 0; i < m; i++){
		double dist = boost::geometry::distance(intersection_point, curve_intersection_points[neighbor_curve_idx][i]);
		if(dist < min_dist){
			neighbor_curve_intpt_idx = i;
			min_dist = dist;
		}
	}

	int neighbor_curve_segment_idx = curve_intersection_segment_idx[neighbor_curve_idx][neighbor_curve_intpt_idx];
	int current_curve_segment_idx = curve_intersection_segment_idx[current_curve_idx][current_curve_intpt_idx];

	if(current_curve_is_up){
		raise_curves(intersection_point, curve_points[neighbor_curve_idx], curve_points[current_curve_idx], neighbor_curve_segment_idx,
				current_curve_segment_idx, radius, inserted_seq[current_curve_idx][current_curve_intpt_idx],
				inserted_seq_end_idx[current_curve_idx][current_curve_intpt_idx].first,
				inserted_seq_end_idx[current_curve_idx][current_curve_intpt_idx].second);
	}
	else{
		raise_curves(intersection_point, curve_points[current_curve_idx], curve_points[neighbor_curve_idx],
				current_curve_segment_idx, neighbor_curve_segment_idx, radius,
				inserted_seq[neighbor_curve_idx][neighbor_curve_intpt_idx],
				inserted_seq_end_idx[neighbor_curve_idx][neighbor_curve_intpt_idx].first,
				inserted_seq_end_idx[neighbor_curve_idx][neighbor_curve_intpt_idx].second);
	}

	raising_status[current_curve_idx][current_curve_intpt_idx] = current_curve_is_up ? RAISING_UP : RAISING_DOWN;
	raising_status[neighbor_curve_idx][neighbor_curve_intpt_idx] = current_curve_is_up ? RAISING_DOWN : RAISING_UP;
}

// Recursively process each curve
void process_one_curve(const std::vector< std::vector<Point2> > &curve_points,
		const std::vector< std::vector<Point2> > &curve_intersection_points,
		const std::vector< std::vector<int> > &intesecting_curve_idx,
		const std::vector< std::vector<int> > &curve_intersection_segment_idx,
		double radius, int current_curve, std::vector< std::vector<RaisingStatus> > &raising_status,
		std::vector< std::vector<std::pair<int, int> > > &inserted_seq_end_idx,
		std::vector< std::vector< std::vector<Point3> > > &inserted_seq)
{
	// Pick the first unprocessed intersection point for the current curve
	int n_int_pts = curve_intersection_points[current_curve].size();

	if(n_int_pts == 0){
		return;
	}

	bool totally_new = true;
	for(int i = 0; i < n_int_pts; i++){
		if(raising_status[current_curve][i] != RAISING_UNDECIDED){
			totally_new = false;
			break;
		}
	}

	std::set<int> next_neighbors;


	if(totally_new){
		// Simply alternate between up and down
		for(int i = 0; i < n_int_pts; i++){
			bool current_curve_up = i % 2  == 0;
			process_intersection_pair(curve_points, curve_intersection_points, curve_intersection_segment_idx, intesecting_curve_idx,
					current_curve, i, current_curve_up, radius, raising_status, inserted_seq_end_idx, inserted_seq);
		}

		next_neighbors.insert(intesecting_curve_idx[current_curve].begin(), intesecting_curve_idx[current_curve].end());
	}
	else{
		// Process intersection points based on its preceeding points

		while(true){
			bool find_points = false;
			for(int i = 0; i < n_int_pts; i++){
				if(raising_status[current_curve][i] != RAISING_UNDECIDED && raising_status[current_curve][(i+1) % n_int_pts] == RAISING_UNDECIDED){
					int current_point_idx = (i+1) % n_int_pts;
					bool current_curve_up = raising_status[current_curve][i] == RAISING_DOWN;
					find_points = true;

					process_intersection_pair(curve_points, curve_intersection_points, curve_intersection_segment_idx, intesecting_curve_idx,
							current_curve, current_point_idx, current_curve_up, radius, raising_status, inserted_seq_end_idx, inserted_seq);

					next_neighbors.insert(intesecting_curve_idx[current_curve][current_point_idx]);

					break;
				}
			}

			if(!find_points){
				break;
			}
		}
	}

	for(std::set<int>::iterator iter = next_neighbors.begin(); iter != next_neighbors.end(); ++ iter){
		process_one_curve(curve_points, curve_intersection_points, intesecting_curve_idx, curve_intersection_segment_idx,
			radius, *iter, raising_status, inserted_seq_end_idx, inserted_seq);
	}
}

void find_covered_vertices(int start_idx, int end_idx, int n_vtx, std::set<int> &covered_idx){
	int cur_idx = start_idx;

	while(true){
		cur_idx = (cur_idx + 1) % n_vtx;
		if(cur_idx == end_idx){
			return;
		}

		covered_idx.insert(cur_idx);
	}
}

void merge_lifted_sequences(const std::vector< std::vector<Point2> > &curve_points,
		const std::vector< std::vector<std::pair<int, int> > > &inserted_seq_end_idx,
		const std::vector< std::vector< std::vector<Point3> > > &inserted_seq,
		double radius,
		std::vector< std::vector<Point3> > &output_curve_points)
{
	int m = curve_points.size();
	output_curve_points.resize(m);


//	std::string test_file_names[] = {
//			std::string("test1.obj"),
//			std::string("test2.obj"),
//			std::string("test3.obj"),
//			std::string("test4.obj"),
//			std::string("test5.obj"),
//			std::string("test6.obj"),
//			std::string("test7.obj"),
//			std::string("test8.obj"),
//			std::string("test9.obj"),
//			std::string("test10.obj"),
//			std::string("test11.obj"),
//			std::string("test12.obj"),
//			std::string("test13.obj"),
//			std::string("test14.obj"),
//			std::string("test15.obj"),
//			std::string("test16.obj"),
//			std::string("test17.obj"),
//			std::string("test18.obj"),
//			std::string("test19.obj"),
//			std::string("test20.obj"),
//			std::string("test21.obj"),
//			std::string("test22.obj"),
//			std::string("test23.obj"),
//			std::string("test24.obj"),
//			std::string("test25.obj"),
//			std::string("test26.obj"),
//			std::string("test27.obj"),
//			std::string("test28.obj"),
//			std::string("test29.obj"),
//			std::string("test30.obj"),
//			std::string("test31.obj"),
//			std::string("test32.obj"),
//			std::string("test33.obj"),
//			std::string("ttest1.obj"),
//			std::string("ttest2.obj"),
//			std::string("ttest3.obj"),
//			std::string("ttest4.obj"),
//			std::string("ttest5.obj"),
//			std::string("ttest6.obj"),
//			std::string("ttest7.obj"),
//			std::string("ttest8.obj"),
//			std::string("ttest9.obj"),
//			std::string("ttest10.obj"),
//			std::string("ttest11.obj"),
//			std::string("ttest12.obj"),
//			std::string("ttest13.obj"),
//			std::string("ttest14.obj"),
//			std::string("ttest15.obj"),
//			std::string("ttest16.obj"),
//			std::string("ttest17.obj"),
//			std::string("ttest18.obj"),
//			std::string("ttest19.obj"),
//			std::string("ttest20.obj"),
//			std::string("ttest21.obj"),
//			std::string("ttest22.obj"),
//			std::string("ttest23.obj"),
//			std::string("ttest24.obj"),
//			std::string("ttest25.obj"),
//			std::string("ttest26.obj"),
//			std::string("ttest27.obj"),
//			std::string("ttest28.obj"),
//			std::string("ttest29.obj"),
//			std::string("ttest30.obj"),
//			std::string("ttest31.obj"),
//			std::string("ttest32.obj"),
//			std::string("ttest33.obj")
//	};
//
//
//	int output_idx = 0;

	for(int i = 0; i < m; i++){
		std::vector< std::pair<int, int> > valid_seq_end_idx;
		std::vector< std::vector<Point3> > valid_inserted_seq;

		int n = inserted_seq_end_idx[i].size();
		for(int j = 0; j < n; j++){
			if(inserted_seq_end_idx[i][j].first >= 0 && inserted_seq_end_idx[i][j].second >= 0){
				valid_seq_end_idx.push_back(inserted_seq_end_idx[i][j]);
				valid_inserted_seq.push_back(inserted_seq[i][j]);
			}
		}

		if(valid_seq_end_idx.empty()){
			augment_whole_sequence(curve_points[i], radius, output_curve_points[i]);
		}
		else{
			int n_vtx = curve_points[i].size();
			std::deque<bool> covered_vtx(n_vtx, false);
			int p = valid_seq_end_idx.size();
			for(int j = 0; j < p; j++){
				int start_idx = valid_seq_end_idx[j].first, end_idx = valid_seq_end_idx[j].second;
				int cur_idx = start_idx;
				while(true){
					cur_idx = (cur_idx + 1) % n_vtx;

					if(cur_idx == end_idx){
						break;
					}

					covered_vtx[cur_idx] = true;
				}
			}

			// Find one sequence with un-covered starting point
			int start_j = -1;
			for(int j = 0; j < p; j++){
				if(!covered_vtx[valid_seq_end_idx[j].first]){
					start_j = j;
					break;
				}
			}

			if(start_j >= 0){
				for(int k = 0; k < p-1; k++){
					int cur_j = (start_j + k) % p;
					int next_j = (cur_j + 1) % p;
					output_curve_points[i].insert(output_curve_points[i].end(), valid_inserted_seq[cur_j].begin(),
							valid_inserted_seq[cur_j].end());
					augment_inbetween_sequence(curve_points[i], covered_vtx, valid_seq_end_idx[cur_j].first, valid_seq_end_idx[cur_j].second, valid_seq_end_idx[next_j].first,
							radius, output_curve_points[i]);
				}

				int last_j = (start_j + p-1)%p;
				output_curve_points[i].insert(output_curve_points[i].end(), valid_inserted_seq[last_j].begin(),
											valid_inserted_seq[last_j].end());
				augment_inbetween_sequence(curve_points[i], covered_vtx, valid_seq_end_idx[last_j].second, valid_seq_end_idx[start_j].first,
						radius, output_curve_points[i]);
			}
			else{
				for(int j = 0; j < p; j++){
					output_curve_points[i].insert(output_curve_points[i].end(), valid_inserted_seq[j].begin(),
							valid_inserted_seq[j].end());
					augment_inbetween_sequence(curve_points[i], covered_vtx, valid_seq_end_idx[j].first, valid_seq_end_idx[j].second, valid_seq_end_idx[(j+1)%p].first,
							radius, output_curve_points[i]);
				}
			}


			//saveOBJ (test_file_names[output_idx++], valid_inserted_seq);
		}
	}
}

void raise_curves(const std::vector< std::vector<Point2> > &curve_points, const std::vector< std::vector<Point2> > &curve_intersection_points,
		const std::vector< std::vector<int> > &intesecting_curve_idx,
		const std::vector< std::vector<int> > &curve_intersection_segment_idx, double radius, std::vector< std::vector<Point3> > &output_curve_points)
{
	// For each intersection piece, make sure the curve alternates between raising and non-raising when being traversed along
	std::vector< std::vector<RaisingStatus> > raising_status;
	std::vector< std::vector<std::pair<int, int> > > inserted_seq_end_idx;
	std::vector< std::vector< std::vector<Point3> > > inserted_seq;

	initialize_sequence_family(curve_intersection_segment_idx, RAISING_UNDECIDED, raising_status);
	initialize_sequence_family(curve_intersection_segment_idx, std::make_pair(-1, -1), inserted_seq_end_idx);
	initialize_sequence_family(curve_intersection_segment_idx, inserted_seq);

	// Depth first traversal of all curves
	for(unsigned int i = 0; i < intesecting_curve_idx.size(); i++){
		if(!intesecting_curve_idx[i].empty()){
			process_one_curve(curve_points, curve_intersection_points, intesecting_curve_idx, curve_intersection_segment_idx,
				radius, i, raising_status, inserted_seq_end_idx, inserted_seq);
			break;
		}
	}

	merge_lifted_sequences(curve_points, inserted_seq_end_idx, inserted_seq, radius, output_curve_points);
}


//template<typename T1, typename T2>
//struct UnorderedPair
//{
//	T1 first;
//	T2 second;
//};


void sort_intersection_info(const std::vector< std::vector<Point2> > &curve_points,
		const std::vector<IntersectionData> &intersection_data,
		std::vector< std::vector<Point2> > &curve_intersection_points,
		std::vector< std::vector<int> > &intesecting_curve_idx,
		std::vector< std::vector<int> > &curve_intersection_segment_idx)
{
	int n_curves = curve_points.size();

	std::vector< std::map< size_t, std::vector< std::pair< size_t, Point2 > > > > intersection_point_info(n_curves);
	for(int i = 0; i < static_cast<int>(intersection_data.size()); i++){
		intersection_point_info[intersection_data[i].poly_index_src][intersection_data[i].segment_index_src].push_back(std::make_pair(intersection_data[i].poly_index_tgt, intersection_data[i].point));
		intersection_point_info[intersection_data[i].poly_index_tgt][intersection_data[i].segment_index_tgt].push_back(std::make_pair(intersection_data[i].poly_index_src, intersection_data[i].point));
	}

	curve_intersection_points.resize(n_curves);
	curve_intersection_segment_idx.resize(n_curves);
	intesecting_curve_idx.resize(n_curves);

	for(int i = 0; i < n_curves; i++){
		for(std::map< size_t, std::vector< std::pair< size_t, Point2 > > >::iterator iter = intersection_point_info[i].begin(); iter != intersection_point_info[i].end(); ++ iter){
			std::vector< std::pair< size_t, Point2 > > &pts = iter->second;
			if(static_cast<int>(pts.size()) > 1){
				// Sort the points according to their distance to the common segment starting point
				std::set< std::pair< double, int > > sorted_pts;
				Point2 start_pt = curve_points[i][iter->first];
				for(int k = 0; k < static_cast<int>(pts.size()); k++){
					double dist = boost::geometry::distance(start_pt, pts[k].second);
					sorted_pts.insert(std::make_pair(dist, k));
				}

				for(std::set<std::pair<double, int> >::iterator iter2 = sorted_pts.begin(); iter2 != sorted_pts.end(); ++ iter2){
					curve_intersection_points[i].push_back(iter->second[iter2->second].second);
					curve_intersection_segment_idx[i].push_back(iter->first);
					intesecting_curve_idx[i].push_back(iter->second[iter2->second].first);
				}
			}
			else{
				curve_intersection_points[i].push_back(iter->second.front().second);
				curve_intersection_segment_idx[i].push_back(iter->first);
				intesecting_curve_idx[i].push_back(iter->second.front().first);
			}
		}
	}
}


void generate_arc_without_duplicate_vertices(const std::vector< std::vector<Point2> > &curve_points,
		const std::vector<IntersectionData> &intersection_data,
		double radius,
		std::vector< std::vector<Point3> > &output_curve_points)
{
	// First sort the intersection points within one curve
	//std::vector< std::set<int> > intersection_info;
	std::vector< std::vector<Point2> > curve_intersection_points;
	std::vector< std::vector<int> > intesecting_curve_idx;
	std::vector< std::vector<int> > curve_intersection_segment_idx;

	sort_intersection_info(curve_points, intersection_data, curve_intersection_points, intesecting_curve_idx, curve_intersection_segment_idx);

	// enumerate all intersection points within one curve, and raise arcs if necessary
	raise_curves(curve_points, curve_intersection_points, intesecting_curve_idx, curve_intersection_segment_idx, radius, output_curve_points);
}


//void check_orientation(const std::vector< std::vector<Point2> > &curve_points)
//{
//	int m = curve_points.size();
//	std::vector<Eigen::Vector3d> normal_vec;
//
//	for(int i = 0; i < m; i++){
//		int n = curve_points[i].size();
//		std::vector<Eigen::Vector3d> edge_vecs;
//		for(int j = 1; j < n; j++){
//			Eigen::Vector3d cur_vec(curve_points[j].x() - curve_points[j-1].x(),
//					curve_points[j].y() - curve_points[j-1].y(), 0.0);
//			cur_vec /= cur_vec.norm();
//			edge_vecs.push_back(cur_vec);
//		}
//
//		double max_innerprod = 0.0;
//		Eigen::Vector3d best_normal(0.0, 0.0, 0.0);
//
//		for(int j = 1; j < n-1; j++){
//			double cur_innerprod = std::abs(edge_vecs[j].dot(edge_vecs[j-1]));
//			if(cur_innerprod > max_innerprod){
//				best_normal = edge_vecs[j].cross(edge_vecs[j-1]);
//				max_innerprod = cur_innerprod;
//			}
//		}
//	}
//}

Point3 mid_point(const Point3 &p1, const Point3 &p2)
{
	return Point3( (p1.get<0>() + p2.get<0>()) * 0.5, (p1.get<1>() + p2.get<1>()) * 0.5,
			(p1.get<2>() + p2.get<2>()) * 0.5 );
}

void remove_closeby_vertices(const std::vector<Point3> &vtx_sequence,
		double eps_dist, std::vector<Point3> &clean_sequence)
{
	std::vector<Point3> cur_seq = vtx_sequence;

	while(true){
		double min_dist = 1e10;
		int min_dist_idx = -1;
		int m = cur_seq.size();

		for(int i = 0; i < m; i++){
			double candidate_length = boost::geometry::distance(cur_seq[i], cur_seq[(i+1)%m]);
			if(candidate_length < min_dist){
				min_dist = candidate_length;
				min_dist_idx = i;
			}
		}

		if(min_dist > eps_dist){
			break;
		}

		Point3 new_point = mid_point(cur_seq[min_dist_idx], cur_seq[(min_dist_idx+1)%m]);

		std::vector<Point3> new_seq;

		// Replace the two close-by points with their mid points
		if(min_dist_idx == m-1){
			new_seq.push_back(new_point);
			new_seq.insert(new_seq.end(), cur_seq.begin()+1, cur_seq.begin() + (m - 1));
		}
		else{
			new_seq.assign(cur_seq.begin(), cur_seq.begin() + min_dist_idx);
			new_seq.push_back(new_point);
			new_seq.insert(new_seq.end(), cur_seq.begin() + (min_dist_idx+2), cur_seq.end());
		}

		int new_size = new_seq.size();
		assert( new_size + 1 == m );
	}

	clean_sequence = cur_seq;
}

void generate_arc(const std::vector< std::vector<Point2> > &curve_points,
		const std::vector<IntersectionData> &intersection_data,
		double radius,
		std::vector< std::vector<Point3> > &output_curve_points)
{
	// Get rid of duplicate end vertices
	std::vector< std::vector<Point2> > simplified_points = curve_points;
	for(unsigned int i = 0; i < simplified_points.size(); i++){
		simplified_points[i].pop_back();
	}

	generate_arc_without_duplicate_vertices(simplified_points,
			intersection_data, radius, output_curve_points);

	double eps_dist = std::min(1e-5, 1e-4 * radius);

	// Add back duplicate points
	for(unsigned int i = 0; i < output_curve_points.size(); i++){
		std::vector<Point3> clean_output_curve;
		remove_closeby_vertices(output_curve_points[i], eps_dist, clean_output_curve);
		clean_output_curve.push_back(clean_output_curve.front());
		output_curve_points[i] = clean_output_curve;
	}
}

}


