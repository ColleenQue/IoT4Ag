def get_lawn_mower_pts(diagonal_corners, num_polygons, direction):
    # align diagonal_corners with regular (x-y) cartesian co-ordinates
    # i.e. treat diagonal_corners[0] as bottom left (start), diagonal_corners[1] as top right
    # (and end for even number of polygons or penultemate if for odd number of polygons)
    # then we have either vertical columns or horizontal columns
    # get the ranges in x and y direction
    x_range = abs(diagonal_corners[0][0] - diagonal_corners[1][0])
    y_range = abs(diagonal_corners[0][1] - diagonal_corners[1][1])
    # get number of waypoints
    num_pts = 4 + (num_polygons - 1) * 2
    # see points in pairs of same y values, exluding the first
    points = [0] * num_pts
    points[0] = diagonal_corners[0]
    # fill in all the rest of points
    if direction == 'vertical':
        for i in range(0, num_pts, 2):
            points[i]     = (diagonal_corners[0][0] + (i/2) * (x_range/num_polygons), diagonal_corners[0][1] + (i % 1) * y_range)
            points[i + 1] = (diagonal_corners[0][0] + (i/2) * (x_range/num_polygons), diagonal_corners[0][1] + (i % 2 + 1) * y_range)
    elif direction == 'horizontal':
        for i in range(0, num_pts, 2):
            points[i]     = (diagonal_corners[0][0] + (i % 1) * x_range, diagonal_corners[0][1] + (i/2) * (y_range/num_polygons))
            points[i + 1] = (diagonal_corners[0][0] + (i % 2 + 1) * x_range, diagonal_corners[0][1] + (i/2) * (y_range/num_polygons))
    # flip every other pair for lawn mower pattern
    for i, pt in enumerate(points):
        if (i + 1) % 4 == 0:
            points[i], points[i-1] = points[i-1], points[i]
    return points





bottom_left = [-9.9,-9.9]
top_right = [9.9,9.9]


diagonal_corners = [bottom_left, top_right]

num_polygons = 4
direction = "vertical"

print(get_lawn_mower_pts(diagonal_corners, num_polygons, direction))