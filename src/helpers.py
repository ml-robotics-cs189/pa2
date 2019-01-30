from shapely.geometry import Polygon, LineString
import math

def min_and_max_y(region, x):
	# Given an x-coordinate, find the largest and smallest y-values
	# such that (x, y') is inside the region.
	# NOTE: this may return a single y-value, if we're right 
	# at the edge of the polygon and there's a single point intersection.
	_, min_y, _, max_y = region.bounds
	x_line = LineString([(x, min_y), (x, max_y)])
	return x_line.intersection(region).xy[1].tolist()

def distance(point_1, point_2):
	x1, y1 = point_1
	x2, y2 = point_2
	return math.hypot(x2 - x1, y2 - y1)


### testing ###
if __name__ == "__main__":
	p = Polygon([(0, 0), (100, 0), (100, 100), (0, 100)])
	min_y, max_y = min_and_max_y(p, 50)
	print(min_y, max_y)