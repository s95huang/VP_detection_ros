import cv2
import math
import numpy as np


class VPDetection:
    def __init__(self, reject_degree_th=4.0):
        self.reject_degree_th = reject_degree_th

    def filter_lines(self, lines):
        final_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            m = (y2 - y1) / (x2 - x1) if x1 != x2 else 1e8
            c = y2 - m * x2
            theta = abs(math.degrees(math.atan(m)))

            if self.reject_degree_th <= theta <= (90 - self.reject_degree_th):
                l = math.hypot(y2 - y1, x2 - x1)
                final_lines.append([x1, y1, x2, y2, m, c, l])

        final_lines = sorted(final_lines, key=lambda x: x[-1], reverse=True)[:min(15, len(final_lines))]
        return final_lines

    def get_lines(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_gray_image = cv2.GaussianBlur(gray_image, (5, 5), 1)
        edge_image = cv2.Canny(blur_gray_image, 40, 255)
        lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180, 50, 10, 15)
        return self.filter_lines(lines) if lines is not None else []

    def get_vanishing_point(self, lines):
        vanishing_point = None
        min_error = float('inf')

        for i, line1 in enumerate(lines[:-1]):
            m1, c1 = line1[4], line1[5]
            for line2 in lines[i + 1:]:
                m2, c2 = line2[4], line2[5]

                if m1 != m2:
                    x0, y0 = (c1 - c2) / (m2 - m1), m1 * ((c1 - c2) / (m2 - m1)) + c1
                    err = sum([math.hypot(y0 - (m * x0 + c), x0 - ((y0 - c) / m)) ** 2 for m, c in [(line[4], line[5]) for line in lines]])
                    err = math.sqrt(err)

                    if min_error > err:
                        min_error = err
                        vanishing_point = [x0, y0]

        return vanishing_point

    def find_vanishing_points(self, image):
        lines = self.get_lines(image)
        vanishing_point = self.get_vanishing_point(lines)
        return vanishing_point
