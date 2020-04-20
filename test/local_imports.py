import sys
import os

# Add python directory to path
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))

# Set display environment variable to avoid CI errors
os.environ['DISPLAY'] = ':0'


def printMessage():
    print("Local imports working")
