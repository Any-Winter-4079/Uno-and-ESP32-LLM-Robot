import os
import sys

def project_root(*subpath):
    """
    Returns an absolute path under the 'computer/' folder
    """
    root = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(root, *subpath)

def resolve_dinov2_path():
    return project_root("depth", "depth_anything", "torchhub", "facebookresearch_dinov2_main")

# Add 'computer/' to sys.path for top-level imports
_root = project_root()  # This is the absolute path to the 'computer' directory
if _root not in sys.path:
    sys.path.insert(0, _root)
