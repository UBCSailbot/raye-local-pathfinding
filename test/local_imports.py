# Add python directory to path
import sys
import os
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))


def printMessage():
    print("Local imports working")
