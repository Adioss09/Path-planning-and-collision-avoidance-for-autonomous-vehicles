import numpy as np
import scipy.interpolate as interpolate

class PathSmoother:
    def __init__(self):
        pass
        
    def smooth(self, path, s=0.0):
        """
        Smooths a list of (x,y) points using B-spline.
        """
        if len(path) < 4:
            return path # Too short to smooth with cubic spline
            
        x = [p[0] for p in path]
        y = [p[1] for p in path]
        
        # Remove duplicates
        unique_pts = []
        for p in path:
            if not unique_pts or unique_pts[-1] != p:
                unique_pts.append(p)
                
        if len(unique_pts) < 4:
            return unique_pts
            
        x = [p[0] for p in unique_pts]
        y = [p[1] for p in unique_pts]
        
        try:
            tck, u = interpolate.splprep([x, y], s=s)
            unew = np.linspace(0, 1, len(path) * 2) # interpolate to more points
            out = interpolate.splev(unew, tck)
            
            smoothed_path = [(float(out[0][i]), float(out[1][i])) for i in range(len(unew))]
            return smoothed_path
        except Exception as e:
            print(f"Smoothing failed: {e}")
            return path
