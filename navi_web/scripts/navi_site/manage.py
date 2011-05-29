#!/usr/bin/python
import roslib; roslib.load_manifest('navi_web')
import rospy

print "arguments :"
import sys
[ sys.stdout.write(' '+ arg) for arg in sys.argv]

sys.argv = sys.argv[0:2]
sys.argv.append('192.168.233.2:8080')


from django.core.management import execute_manager
try:
    import settings # Assumed to be in the same directory.
except ImportError:
    import sys
    sys.stderr.write("Error: Can't find the file 'settings.py' in the directory containing %r. It appears you've customized things.\nYou'll have to run django-admin.py, passing it your settings module.\n(If the file settings.py does indeed exist, it's causing an ImportError somehow.)\n" % __file__)
    sys.exit(1)

if __name__ == "__main__":
    execute_manager(settings)
