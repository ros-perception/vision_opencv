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

# Sphinx configuration for the vision_opencv metapackage documentation.
#
# This file is not used directly: rosdoc2 exec()s it from a generated conf.py
# and then adds (or overrides) settings such as the project name and the
# version (both taken from package.xml). Build the documentation with:
#
#   rosdoc2 build --package-path <path-to-this-package>

from datetime import datetime

# -- General configuration ----------------------------------------------------

extensions = [
    'sphinx_rtd_theme',
]

source_suffix = '.rst'
master_doc = 'index'

# rosdoc2 copies this directory a second time under 'user_docs/' and generates
# 'user_docs.rst' / '__readme_include.rst' wrappers for packages that do not
# ship their own index. This one does, so exclude the duplicate tree.
exclude_patterns = ['user_docs', 'user_docs.rst', 'user_docs_tutorials.rst',
                    '__readme_include.rst']

project = 'vision_opencv'
author = 'Open Source Robotics Foundation, Inc.'
copyright = '2009-' + datetime.now().strftime('%Y') + ', ' + author  # noqa: A001

pygments_style = 'sphinx'

# -- Options for HTML output --------------------------------------------------

html_theme = 'sphinx_rtd_theme'
htmlhelp_basename = 'vision_opencv_doc'
