#include "utils.h"
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

bool
veiv::saveOBJ (std::string &filename,
               std::vector<std::vector<Point3> > &model)
{
  std::ofstream out_file (filename.c_str ());
  if (!out_file.is_open ())
  {
    fprintf (stderr, "Error writing to file %s.\n", filename.c_str ());
    return (false);
  }

  /// Write all the vertices
  for (size_t poly_i = 0; poly_i < model.size (); ++poly_i)
  {
    for (size_t p_i = 0; p_i < model[poly_i].size (); ++p_i)
      out_file << "v " << model[poly_i][p_i].get<0> () << " " << model[poly_i][p_i].get<1> () << " " << model[poly_i][p_i].get<2> () << std::endl;
  }

  /// Write the polygons
  size_t index = 0;
  for (size_t poly_i = 0; poly_i < model.size (); ++poly_i)
  {
    out_file << "f ";
    for (size_t p_i = 0; p_i < model[poly_i].size (); ++p_i)
      out_file << (index++ + 1) << " ";
    out_file << std::endl;
  }

  return (true);
}



bool
veiv::extrudeCircleAlongPath (const std::vector<std::vector<Point3> > &model,
                              const double radius,
                              const size_t num_circle_samples,
                              const double sampling_dist,
                              std::vector<std::vector<Point3> > &result)
{
  /// HACK!!!
  result.push_back (std::vector<Point3> ());


  /// Generate the circle points
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > circle_samples;
  for (size_t i = 0; i < num_circle_samples; ++i)
  {
    Eigen::Vector4d p;
    p[0] = radius * cos (i * 2 * M_PI / num_circle_samples);
    p[1] = radius * sin (i * 2 * M_PI / num_circle_samples);
    p[2] = 0.;
    p[3] = 1.;

    circle_samples.push_back (p);
  }


  /// For each polygon
  for (size_t poly_i = 0; poly_i < model.size (); ++poly_i)
  {
    /// Compute the tangents along the curve
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tangents;
    for (int p_i = 0; p_i < model[poly_i].size (); ++p_i)
    {
      Eigen::Vector3d curr (model[poly_i][ p_i      % model[poly_i].size ()].get<0> (),
                            model[poly_i][ p_i      % model[poly_i].size ()].get<1> (),
                            model[poly_i][ p_i      % model[poly_i].size ()].get<2> ());
      Eigen::Vector3d next (model[poly_i][(p_i + 1) % model[poly_i].size ()].get<0> (),
                            model[poly_i][(p_i + 1) % model[poly_i].size ()].get<1> (),
                            model[poly_i][(p_i + 1) % model[poly_i].size ()].get<2> ());

      Eigen::Vector3d tangent = (curr - next).normalized ();
      tangents.push_back (tangent);
    }

    /// Smooth the tangents
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tangents_smoothed;
    for (int p_i = 0; p_i < model[poly_i].size (); ++p_i)
      tangents_smoothed.push_back ((tangents[(p_i + tangents.size () - 1) % tangents.size ()] + tangents[p_i]).normalized ());

    for (int p_i = 0; p_i < model[poly_i].size (); ++p_i)
    {
      Eigen::Vector3d curr (model[poly_i][ p_i      % model[poly_i].size ()].get<0> (),
                            model[poly_i][ p_i      % model[poly_i].size ()].get<1> (),
                            model[poly_i][ p_i      % model[poly_i].size ()].get<2> ());
      Eigen::Vector3d next (model[poly_i][(p_i + 1) % model[poly_i].size ()].get<0> (),
                            model[poly_i][(p_i + 1) % model[poly_i].size ()].get<1> (),
                            model[poly_i][(p_i + 1) % model[poly_i].size ()].get<2> ());
      Eigen::Vector3d next2(model[poly_i][(p_i + 2) % model[poly_i].size ()].get<0> (),
                            model[poly_i][(p_i + 2) % model[poly_i].size ()].get<1> (),
                            model[poly_i][(p_i + 2) % model[poly_i].size ()].get<2> ());

      std::cout << "next:\n" << next << "\nnext2:\n" << next2 << "\n";

      Eigen::Vector3d curr_tangent = tangents_smoothed[p_i];
      Eigen::Vector3d next_tangent = tangents_smoothed[(p_i + 1) % model[poly_i].size ()];

      double segment_length = (next - curr).norm ();
      size_t num_segment_samples = segment_length / sampling_dist;
      for (size_t s_i = 0; s_i < num_segment_samples; ++s_i)
      {
        /// Compute the weighted tangent
        Eigen::Vector3d center = (next * static_cast<double> (s_i)  +
                                  curr * static_cast<double> (num_segment_samples - s_i)) / static_cast<double> (num_segment_samples);

        Eigen::Vector3d tangent = ((next_tangent * static_cast<double> (s_i) +
                                   curr_tangent * static_cast<double> (num_segment_samples - s_i)) / static_cast<double> (num_segment_samples)).normalized ();
//        Eigen::Vector3d tangent = curr_tangent;

        Eigen::Matrix4d transf (Eigen::Matrix4d::Identity ());
        /// Align the normal of the circle plane with the tangent
        transf.block<3, 1> (0, 0) = tangent.unitOrthogonal ();
        transf.block<3, 1> (0, 1) = tangent.cross (transf.block<3, 1> (0, 0)).normalized ();
        transf.block<3, 1> (0, 2) = tangent;

        /// Center the circle at the point
        transf.block<3, 1> (0, 3) = center;

        std::cout << "transf:\n" << transf << "\n";

        /// Now move the circle points to their new location
        for (size_t c_i = 0; c_i < circle_samples.size (); ++c_i)
        {
          Eigen::Vector4d point = transf * circle_samples[c_i];

          /// HACK!!!
          Point3 p (point[0], point[1], point[2]);
          result.back ().push_back (p);
        }
      }
    }
  }

  return (true);
}
