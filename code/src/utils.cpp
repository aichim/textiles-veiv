#include "utils.h"
#include <cstdio>
#include <sys/stat.h>
#include "svgtiny.h"

namespace veiv {

    void render_path(struct svgtiny_shape *path);

    void new_path();
    void close_path();
    void move_to(float x, float y);
    void line_to(float x, float y);
    void curve_to(float x1, float y1,
                  float x2, float y2,
                  float x3, float y3);
}

bool
veiv::readSVG (const std::string &filename, Drawing &drawing) {
    FILE *fd;
	struct stat sb;
	char *buffer;
	size_t size;
	size_t n;
	svgtiny_code code;

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
			veiv::render_path(&diagram->shape[i]);

		} else if (diagram->shape[i].text) {
			printf("Ignoring Text Node\n");
		}
	}
    
	svgtiny_free(diagram);
	return true;
}


void veiv::render_path(struct svgtiny_shape *path) {
	new_path();
	for (int j = 0; j != path->path_length; ) {
		switch ((int) path->path[j]) {
            case svgtiny_PATH_MOVE:
                veiv::move_to( path->path[j + 1],
                         path->path[j + 2]);
                j += 3;
                break;
            case svgtiny_PATH_CLOSE:
                veiv::close_path();
                j += 1;
                break;
            case svgtiny_PATH_LINE:
                veiv::line_to( path->path[j + 1],
                         path->path[j + 2]);
                j += 3;
                break;
            case svgtiny_PATH_BEZIER:
                veiv::curve_to(      path->path[j + 1],
                          path->path[j + 2],
                          path->path[j + 3],
                          path->path[j + 4],
                          path->path[j + 5],
                          path->path[j + 6]);
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

void veiv::new_path() {
    printf("Starting new Path\n");
}
void veiv::close_path() {
    printf("Closing Path\n\n");
}

void veiv::move_to(float x, float y) {
    printf("Move to (%.1f, %.1f)\n", x, y);
}
void veiv::line_to(float x, float y) {
    printf("Line to (%.1f, %.1f)\n", x, y);
}
void veiv::curve_to(float x1, float y1,
              float x2, float y2,
              float x3, float y3) {
    printf("Bezier Segment\n");
    printf("  (%.1f, %.1f)\n", x1, y1);
    printf("  (%.1f, %.1f)\n", x2, y2);
    printf("  (%.1f, %.1f)\n", x3, y3);
}



