#ifndef _ARC_GENERATOR_H_
#define _ARC_GENERATOR_H_

#include <boost/geometry.hpp>
#include "drawing.h"
#include <vector>
#include <set>
#include <map>
#include <utility>
#include <Eigen/Dense>
#include <deque>
#include <cmath>

namespace veiv
{
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3;
typedef Point2d Point2;

//#define OVERHANG_ANGLE 50
//#define LOWER_RADIUS_RATIO 6
//#define UPPER_RADIUS_RATIO 2
//
//
//void
//to_eigen_2d (const Point2 &pt, Eigen::Vector2d &eigen_pt);
//
//Point2
//interpolate_point2 (const Point2 &pt1, const Point2 &pt2, double val1, double val2, double target_val);
//
//Point3
//augmented_point2 (const Point2 &pt, double z);
//
//// Use SVD to figure out the pricipal direction with small variance
//void
//PCA_2D (const std::vector<Point2> &pts, Point2 &centroid, Point2 &dir);
//
//void
//sample_lower_curve (const std::vector<Point2> &curve, const Point2 &mid_point, int segment_idx, double target_length, std::deque<Point2> &pts);
//
//int
//find_interval(const std::vector<double> &val, double value);
//
//void
//augment_inbetween_sequence (const std::vector<Point2> &curve_seq, int prev_end_idx, int next_start_idx, std::vector<Point3> &augmented_sequence);
//
//void
//raise_single_sequence(const std::vector<Point2> &curve_seq, double radius, std::vector<Point3> &lifted_sequence);
//
//void
//sample_upper_curve (const Point2 &int_pt, const Point2 &principal_dir,
//                    const std::deque<Point2> &lower_sample, const std::vector<Point2> &upper_curve,
//                    double radius,
//                    int upper_segment_idx,
//                    std::vector<Point3> &lifted_upper_samples,
//                    int &upper_start_idx, int &upper_end_idx);
//
//void
//raise_curves (const Point2 &int_point, const std::vector<Point2> &lower_curve, const std::vector<Point2> &upper_curve,
//              int lower_curve_segment_idx, int upper_curve_segment_idx, double radius, std::vector<Point3> &lifted_upper_samples,
//              int &upper_start_idx, int &upper_end_idx);
//
//// Three possible statuses in the process of determining raising
//enum RaisingStatus{
//  RAISING_UNDECIDED,
//  RAISING_UP,
//  RAISING_DOWN
//};
//
//
//template<typename T1, typename T2>
//void initialize_sequence_family (const std::vector< std::vector<T1> > &ref_sequence, std::vector< std::vector<T2> > &output_seq);
//
//template<typename T1, typename T2>
//void initialize_sequence_family (const std::vector< std::vector<T1> > &ref_sequence, const T2 &init_value, std::vector< std::vector<T2> > &output_seq);
//
//void
//process_intersection_pair (const std::vector< std::vector<Point2> > &curve_points,
//                           const std::vector< std::vector<Point2> > &curve_intersection_points,
//                           const std::vector< std::vector<int> > &curve_intersection_segment_idx,
//                           const std::vector< std::vector<int> > &intesecting_curve_idx,
//                           int current_curve_idx,
//                           int current_curve_intpt_idx, bool current_curve_is_up, double radius,
//                           std::vector< std::vector<RaisingStatus> > &raising_status,
//                           std::vector< std::vector<std::pair<int, int> > > &inserted_seq_end_idx,
//                           std::vector< std::vector< std::vector<Point3> > > &inserted_seq);
//
//// Recursively process each curve
//void
//process_one_curve (const std::vector< std::vector<Point2> > &curve_points,
//                   const std::vector< std::vector<Point2> > &curve_intersection_points,
//                   const std::vector< std::vector<int> > &intesecting_curve_idx,
//                   const std::vector< std::vector<int> > &curve_intersection_segment_idx,
//                   double radius, int current_curve, std::vector< std::vector<RaisingStatus> > &raising_status,
//                   std::vector< std::vector<std::pair<int, int> > > &inserted_seq_end_idx,
//                   std::vector< std::vector< std::vector<Point3> > > &inserted_seq);
//
//void
//merge_lifted_sequences (const std::vector< std::vector<std::pair<int, int> > > &inserted_seq_end_idx,
//                        const std::vector< std::vector< std::vector<Point3> > > &inserted_seq,
//                        std::vector< std::vector<Point3> > &output_curve_points);
//
//void
//raise_curves (const std::vector< std::vector<Point2> > &curve_points, const std::vector< std::vector<Point2> > &curve_intersection_points,
//              const std::vector< std::vector<int> > &intesecting_curve_idx,
//              const std::vector< std::vector<int> > &curve_intersection_segment_idx, double radius, std::vector< std::vector<Point3> > &output_curve_points);
//
//void
//sort_intersection_info (const std::vector< std::vector<Point2> > &curve_points,
//                        const std::vector<IntersectionData> &intersection_data,
//                        std::vector< std::vector<Point2> > &curve_intersection_points,
//                        std::vector< std::vector<int> > &intesecting_curve_idx,
//                        std::vector< std::vector<int> > &curve_intersection_segment_idx);

void generate_arc (const std::vector< std::vector<Point2> > &curve_points,
                   const std::vector<IntersectionData> &intersection_data,
                   double radius,
                   std::vector< std::vector<Point3> > &output_curve_points);

}

#endif
