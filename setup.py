import sys
from distutils.core import setup
from xml.etree.ElementTree import ElementTree

# get the stack info from stack.xml
stackinfo = {}
for tagname in ['name', 'version', 'author', 'url', 'license']:
    try:
        root = ElementTree(None, 'stack.xml')
        stackinfo[tagname] = root.findtext(tagname)
    except Exception, e:
        print >> sys.stderr, 'Could not extract %s from your stack.xml:\n%s' % (tagname, e)
        sys.exit(-1)

setup(packages=['cv_bridge'],
      package_dir= {
          'cv_bridge' : 'cv_bridge/src/cv_bridge'
          },
      **stackinfo
      )
