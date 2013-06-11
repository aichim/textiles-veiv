/*
 * This file is part of Libsvgtiny
 * Licensed under the MIT License,
 *                http://opensource.org/licenses/mit-license.php
 * Copyright 2009-2010 James Bursa <james@semichrome.net>
 */

/*
 * This example loads an SVG using libsvgtiny and then displays it in an X11
 * window using cairo.
 *
 * Lum June 2013
 *  - I removed X11 and Cairo References
 */

#include <stdbool.h>
#include <stdio.h>
#include <sys/stat.h>
#include "svgtiny.h"

struct svgtiny_diagram *diagram;
char *svg_path;
float scale = 1.0;

void render_path(float scale, struct svgtiny_shape *path);

// Rendering Callbacks
void new_path();
void close_path();
void move_to(float x, float y);
void line_to(float x, float y);
void curve_to(float x1, float y1,
              float x2, float y2,
              float x3, float y3);


/**
 * Main program.
 */
int main(int argc, char *argv[]) {
	FILE *fd;
	struct stat sb;
	char *buffer;
	size_t size;
	size_t n;
	svgtiny_code code;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s FILE\n", argv[0]);
		return 1;
	}
	svg_path = argv[1];

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

	buffer = malloc(size);
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
	diagram = svgtiny_create();
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
			render_path(scale, &diagram->shape[i]);

		} else if (diagram->shape[i].text) {
			printf("Ignoring Text Node\n");
		}
	}

	svgtiny_free(diagram);
	return 0;
}

void render_path(float scale, struct svgtiny_shape *path) {
	new_path();
	for (int j = 0; j != path->path_length; ) {
		switch ((int) path->path[j]) {
            case svgtiny_PATH_MOVE:
                move_to(scale * path->path[j + 1],
                        scale * path->path[j + 2]);
                j += 3;
                break;
            case svgtiny_PATH_CLOSE:
                close_path();
                j += 1;
                break;
            case svgtiny_PATH_LINE:
                line_to(scale * path->path[j + 1],
                        scale * path->path[j + 2]);
                j += 3;
                break;
            case svgtiny_PATH_BEZIER:
                curve_to(     scale * path->path[j + 1],
                         scale * path->path[j + 2],
                         scale * path->path[j + 3],
                         scale * path->path[j + 4],
                         scale * path->path[j + 5],
                         scale * path->path[j + 6]);
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

void new_path() {
    printf("Starting new Path\n");
}
void close_path() {
    printf("Closing Path\n\n");
}

void move_to(float x, float y) {
    printf("Move to (%.1f, %.1f)\n", x, y);
}
void line_to(float x, float y) {
    printf("Line to (%.1f, %.1f)\n", x, y);
}
void curve_to(float x1, float y1,
              float x2, float y2,
              float x3, float y3) {
    printf("Bezier Segment\n");
    printf("  (%.1f, %.1f)\n", x1, y1);
    printf("  (%.1f, %.1f)\n", x2, y2);
    printf("  (%.1f, %.1f)\n", x3, y3);
}
