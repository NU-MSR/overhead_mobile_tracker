from math import pi, fmod

# map to 0 to 2pi
def normalize_angle_positive(angle):
    """
    map an angle from 0 to 2pi
    """
    return fmod(fmod(angle, 2.0*pi) + 2.0*pi, 2.0*pi)

# map to -pi to +pi
def normalize_angle(angle):
    """
    map an angle to -pi to pi
    """
    a = normalize_angle_positive(angle)
    if a > pi: a -= 2.0*pi
    return a

# utility function for getting minimum angular distance between two angles in
# radians.  Answer will be -pi <= result <= pi, adding result to 'before' will
# always be an angle equivalent to 'after'
def shortest_angular_distance(before, after):
    """
    Utility function for getting minimum angular distance between two angles in
    radians.  Answer will be -pi <= result <= pi, adding result to 'before' will
    always be an angle equivalent to 'after'
    """
    result = normalize_angle_positive(normalize_angle_positive(after) -
                                      normalize_angle_positive(before))
    if result > pi:
        result = -(2.0*pi-result)
    return normalize_angle(result)
