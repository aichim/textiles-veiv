#include "SVGReader.h"

#include <cstdio>
#include <sys/stat.h>
#include "svgtiny.h"

veiv::SVGReader::SVGReader ()
{}

bool
veiv::SVGReader::readFile (const std::string & filename, Drawing::Ptr drawing)
{
  FILE *fd;
  struct stat sb;
  char *buffer;
  size_t size;
  size_t n;
  svgtiny_code code;

  /* Note: I don't reset this pointer at the end of the function so it 
           holds the references until the reader is deallocated.
           Should fix this ...
   */
  this->drawing_ = drawing;

  const char * svg_path = filename.c_str();

  /* load file into memory buffer */
  fd = fopen(svg_path, "rb");
  if (!fd) {
    perror(svg_path);
    return 1;
  }

  if (stat(svg_path, &sb)) {
    perror(svg_path);
    return 1;
  }
  size = sb.st_size;

  buffer = static_cast<char *>(malloc(size));
  if (!buffer) {
    fprintf(stderr, "Unable to allocate %lld bytes\n",
            (long long) size);
    return 1;
  }

  n = fread(buffer, 1, size, fd);
  if (n != size) {
    perror(svg_path);
    return 1;
  }

  fclose(fd);

  /* create svgtiny object */
  struct svgtiny_diagram * diagram = svgtiny_create();
  if (!diagram) {
    fprintf(stderr, "svgtiny_create failed\n");
    return 1;
  }

  /* parse */
  code = svgtiny_parse(diagram, buffer, size, svg_path, 1000, 1000);
  if (code != svgtiny_OK) {
    fprintf(stderr, "svgtiny_parse failed: ");
    switch (code) {
      case svgtiny_OUT_OF_MEMORY:
        fprintf(stderr, "svgtiny_OUT_OF_MEMORY");
        break;
      case svgtiny_LIBXML_ERROR:
        fprintf(stderr, "svgtiny_LIBXML_ERROR");
        break;
      case svgtiny_NOT_SVG:
        fprintf(stderr, "svgtiny_NOT_SVG");
        break;
      case svgtiny_SVG_ERROR:
        fprintf(stderr, "svgtiny_SVG_ERROR: line %i: %s",
                diagram->error_line,
                diagram->error_message);
        break;
      default:
        fprintf(stderr, "unknown svgtiny_code %i", code);
        break;
    }
    fprintf(stderr, "\n");
  }

  free(buffer);

  printf("viewbox 0 0 %u %u\n",
         diagram->width, diagram->height);


  for (int i = 0; i != diagram->shape_count; i++) {
    if (diagram->shape[i].path) {
      this->render_path(&diagram->shape[i]);

    } else if (diagram->shape[i].text) {
      printf("Ignoring Text Node\n");
    }
  }

  svgtiny_free(diagram);
  return true;
}

void veiv::SVGReader::render_path(struct svgtiny_shape *path) {
  new_path();
  for (int j = 0; j != path->path_length; ) {
    switch ((int) path->path[j]) {
      case svgtiny_PATH_MOVE:
        this->move_to( Point2d(path->path[j + 1], path->path[j + 2]) );
        j += 3;
        break;
      case svgtiny_PATH_CLOSE:
        this->close_path();
        j += 1;
        break;
      case svgtiny_PATH_LINE:
        this->line_to( Point2d(path->path[j + 1], path->path[j + 2]));
        j += 3;
        break;
      case svgtiny_PATH_BEZIER:
        this->curve_to( Point2d(path->path[j + 1], path->path[j + 2]),
                       Point2d(path->path[j + 3], path->path[j + 4]),
                       Point2d(path->path[j + 5], path->path[j + 6]));
        j += 7;
        break;
      default:
        printf("error ");
        j += 1;
    }
  }
  if (path->fill != svgtiny_TRANSPARENT) {
    //svgtiny_colour color =  path->fill;
    printf("Ignoring Color of Path\n");
  }
  if (path->stroke != svgtiny_TRANSPARENT) {
    //svgtiny_colour stroke = path->stroke;
    printf("Ignoring Stroke of Path\n");
  }
}


void veiv::SVGReader::new_path()
{
  Polygon new_poly;
  drawing_->polygons_.push_back (new_poly);
}

void veiv::SVGReader::close_path()
{
  /// Re-add the last point to close the curve
  boost::geometry::append (drawing_->polygons_.back (), drawing_->polygons_.back ().outer ().front ());
}

void
veiv::SVGReader::move_to(const Point2d & p)
{
  line_to (p);
}

void
veiv::SVGReader::line_to(const Point2d & p)
{
  boost::geometry::append (drawing_->polygons_.back (), p);
}


void veiv::SVGReader::curve_to(const Point2d & p1,
                               const Point2d & p2,
                               const Point2d & p3) {
}
