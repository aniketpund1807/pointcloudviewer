import numpy as np

def find_best_fitting_plane(points):
    """Find the best fitting plane for given points"""
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered)
    normal = vh[2]  # The third row is the normal to the best-fit plane
    return centroid, normal