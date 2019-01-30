#!/usr/bin/python2

from shapely.geometry import Polygon
import numpy as np
from helpers import min_and_max_y


def generate_waypoints_updown(region, lane_count=10):
	# Generates waypoints in an up-and-down lawnmower pattern.
	
	(min_x, _, max_x, _) = region.bounds
	x_coords = np.linspace(min_x, max_x, num=lane_count)
	going_up = True
	waypoints = []
	for x_coord in x_coords:

		y_intersections = min_and_max_y(region, x_coord)
		
		# does this x-line intersect at just a single point?
		if len(y_intersections) == 1:
			points = [(x_coord, y_intersections[0])]
		# if intersects twice, go up (or down) the lane:
		else:
			y_min = y_intersections[0]
			y_max = y_intersections[1]
			points = [(x_coord, y_min), (x_coord, y_max)]
			if not going_up:
				points.reverse()
		
		waypoints += points
		going_up = not going_up	# go the opposite direction in next lane
	
	return waypoints





def get_catalina_region(normalized=False):
	# normalized: make everything positive (and large enough to view)
	catalina = [(33.44481,-118.48498),(33.44443,-118.48435),(33.44469,-118.48413),(33.44507,-118.48474)]
	if normalized:
		c_fixed = []
		for x, y in catalina:
			x_new = ((x * 10000) % 100 - 44) * 50
			y_new = ((y * 10000) % 100 - 50) * 50
			c_fixed.append((x_new, y_new))
		catalina = c_fixed
	return Polygon(catalina)



### TESTING ###

if __name__ == "__main__":
    p = Polygon([(0, 0), (100, 0), (100, 100), (0, 100)])
    w = generate_waypoints_updown(p)
    print(w)

    catalina = get_catalina_region()
    w = generate_waypoints_updown(catalina)
    print(w)

    p = Polygon([(-50, 50), (0, 0), (100, 0), (100, 100), (0, 100)])
    w = generate_waypoints_updown(p)
    print(w)
