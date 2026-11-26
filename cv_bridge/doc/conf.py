# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Open Source Robotics Foundation, Inc. nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Sphinx configuration for the cv_bridge package documentation.
#
# This file is not used directly: rosdoc2 exec()s it from a generated conf.py
# and then adds (or overrides) settings such as the project name, the version
# (both taken from package.xml), the theme and the breathe/exhale configuration
# used for the C++ API. Build the documentation with:
#
#   rosdoc2 build --package-path <path-to-this-package>

from datetime import datetime
import os
import re
import sys

# Put this package's Python sources on sys.path so autodoc can import them.
#
# rosdoc2 only does this for packages that let it generate conf.py. Because
# rosdoc2 exec()s this file from a generated conf.py inside the build tree,
# __file__ points there rather than at doc/, so recover the real location from
# that wrapper's exec() call.
_conf_dir = os.path.dirname(os.path.abspath(__file__))
_doc_dir = _conf_dir
_wrapper = os.path.join(_conf_dir, 'conf.py')
if os.path.isfile(_wrapper):
    with open(_wrapper, encoding='utf-8') as _f:
        _match = re.search(r'exec\(open\("(.+?)"\)', _f.read())
    if _match:
        _doc_dir = os.path.dirname(_match.group(1))
_src_dir = os.path.join(os.path.dirname(_doc_dir), 'python')
if os.path.isdir(os.path.join(_src_dir, 'cv_bridge')) and _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

# -- General configuration ----------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx_rtd_theme',
]

source_suffix = '.rst'
master_doc = 'index'

# rosdoc2 copies this directory a second time under 'user_docs/' and generates
# 'user_docs.rst' / '__readme_include.rst' wrappers for packages that do not
# ship their own index. This one does, and index.rst already links the
# tutorials and the API references, so exclude the duplicate tree.
exclude_patterns = ['user_docs', 'user_docs.rst', 'user_docs_tutorials.rst',
                    '__readme_include.rst']

project = 'cv_bridge'
author = 'Open Source Robotics Foundation, Inc.'
copyright = '2009-' + datetime.now().strftime('%Y') + ', ' + author  # noqa: A001

pygments_style = 'sphinx'

# -- Options for autodoc ------------------------------------------------------

# Mock the dependencies of the Python module, including its compiled extension,
# so the documentation can be built without building the package.
autodoc_mock_imports = ['cv2', 'cv_bridge.boost', 'numpy', 'sensor_msgs']

autodoc_default_options = {
    'members': True,
    'undoc-members': True,
}

# -- Options for HTML output --------------------------------------------------

html_theme = 'sphinx_rtd_theme'
htmlhelp_basename = 'cv_bridge_doc'

# -- Options for intersphinx --------------------------------------------------

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
}
