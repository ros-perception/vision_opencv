import sys
from distutils.core import setup
from xml.etree.ElementTree import ElementTree

# get the stack info from stack.xml
stackinfo = {}
for tagname in ['name', 'version', 'author', 'url', 'license']:
    try:
        root = ElementTree(None, 'package.xml')
        stackinfo[tagname] = root.findtext(tagname)
    except Exception, e:
        print >> sys.stderr, 'Could not extract %s from your package.xml:\n%s' % (tagname, e)
        sys.exit(-1)

setup(
    packages = [
        'cv_bridge',
        ],
    package_dir = {
        'cv_bridge' : 'src/cv_bridge',
        },
    **stackinfo
    )
