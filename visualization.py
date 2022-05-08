"""Visualization.py - Singleton module containing methods used to visualize program output."""


import cv2
import numpy

from obstacle_map import IN_OBSTACLE, IN_MARGIN


OBSTACLE_COLOR = (33, 67, 101)
MARGIN_COLOR = (95, 153, 211)
PATH_COLOR = (20, 255, 57)
FREE_UNEXPLORED_COLOR = (230, 216, 173)
FREE_EXPLORED_COLOR = (150, 146, 92)
LINE_THICKNESS = 1

FRAMES_TO_SKIP = 0

OUTPUT_VIDEO_NAME = "output.avi"
OUTPUT_FPS = 30

VISUALIZE = True

GSD = 1


class PathPlanningVisualizer(object):
    def __init__(self):
        self.canvas = None
        self.output_video = None

        self.image_counter = 0


    def draw_obstacle_map(self, obstacle_map):
        canvas_height = obstacle_map.height * GSD
        canvas_width = obstacle_map.width * GSD
        self.canvas = numpy.zeros((canvas_height, canvas_width, 3), dtype=numpy.uint8)
        for x in range(canvas_width):
            for y in range(canvas_height):

                obstacle_state = obstacle_map.is_coordinate_occupied(obstacle_map.img, x//GSD, y//GSD)
                is_inside_brain = obstacle_map.is_coordinate_inside(x//GSD, y//GSD)
                if obstacle_state or not is_inside_brain:
                    color = OBSTACLE_COLOR
                else:
                    color = FREE_UNEXPLORED_COLOR

                canvas_y = canvas_height - y - 1
                self.canvas[y, x] = color

        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.output_video = cv2.VideoWriter(OUTPUT_VIDEO_NAME, fourcc, OUTPUT_FPS, (self.canvas.shape[1], self.canvas.shape[0]))

        self._record_to_video()


    def draw_visited_node(self, visited_nodes):
        for visited_node in visited_nodes:
            x, y = visited_node.coordinates

            x = int(x * GSD)
            y = int(y * GSD)

            canvas_y = self.canvas.shape[0] - y - 1
            new_coord = (x, y)

            if visited_node.parent_node:
                parent_x = int(visited_node.parent_node.coordinates[0] * GSD)
                parent_y = int(visited_node.parent_node.coordinates[1] * GSD)
                # parent_coord = (parent_x, self.canvas.shape[0] - parent_y - 1)
                parent_coord = (parent_x, parent_y)

                cv2.line(self.canvas, new_coord, parent_coord, FREE_EXPLORED_COLOR, LINE_THICKNESS)
                # self.canvas[canvas_y, x] = FREE_EXPLORED_COLOR

                self.image_counter += 1

        if self.image_counter % (FRAMES_TO_SKIP+1) == 0:
            if VISUALIZE:
                cv2.imshow("Canvas", self.canvas)
                cv2.waitKey(1)

            self._record_to_video()


    def draw_path(self, path):
        last_point_drawn = None

        for node in path:
            x, y = node.coordinates

            x = int(x * GSD)
            y = int(y * GSD)

            canvas_y = self.canvas.shape[0] - y - 1
            this_point = (x, y)

            if last_point_drawn:
                cv2.line(self.canvas, last_point_drawn, this_point, PATH_COLOR, LINE_THICKNESS)

            last_point_drawn = this_point

        self._record_to_video()


    def _record_to_video(self):
        self.output_video.write(self.canvas)


    def write_video_file(self):
        # Linger on the last frame of the video
        for _ in range(3 * OUTPUT_FPS):
            self._record_to_video()
        self.output_video.release()


if __name__ == "__main__":
    pass