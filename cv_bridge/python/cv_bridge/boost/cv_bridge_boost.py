"""
Deprecated location of the C++ part of cv_bridge.

The bindings no longer use Boost.Python. Use ``cv_bridge.cvtColorForDisplay``,
``cv_bridge.getCvType`` and :class:`cv_bridge.CvBridge` instead.
"""
import warnings

from cv_bridge._cv_bridge import CV_MAT_CNWrap  # noqa: F401
from cv_bridge._cv_bridge import CV_MAT_DEPTHWrap  # noqa: F401
from cv_bridge._cv_bridge import cvtColor2  # noqa: F401
from cv_bridge._cv_bridge import cvtColorForDisplay  # noqa: F401
from cv_bridge._cv_bridge import getCvType  # noqa: F401

warnings.warn(
    'cv_bridge.boost.cv_bridge_boost is deprecated and will be removed, import from cv_bridge '
    'instead', DeprecationWarning, stacklevel=2)
