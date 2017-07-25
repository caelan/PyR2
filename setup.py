#!/usr/bin/env python2

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import sys
import os
import numpy

# Directories to include for all imports in cython files
# When convert to using fully qualified paths in all redux files
# can test taking out the include dirs

# Directories to exclude when searching for cython files (.pyx)
exclude_dirs = {'.git', 'build', 'iiwa_description-master'}

# get the path to the input file
def getPath(filename):
    path = os.getcwd()
    for root, dirs, files in os.walk(path, topdown=True):
        if filename in files:
            return root

# checks if any of the exclude directories are in the given path (root)
def inRoot(root):
    inRoot = False
    reduxPath = root.replace(os.getcwd(), '')
    for exclude_dir in exclude_dirs:
        if reduxPath.find(exclude_dir) != -1:
            inRoot = True
    return inRoot

def convert(filename, path, i):
    # this defines the path where the .so files end up
    rootPath = path.replace(os.getcwd() + '/', '')
    modulePath = rootPath.replace('/','.') + '.'

    name = filename[:i]
    setup(
      cmdclass = {'build_ext': build_ext},
      ext_modules=[
      Extension(modulePath + name,
                sources=[path + '/' + filename],
                include_dirs = ['.', numpy.get_include()],
                extra_compile_args=["-O3"])])

def build_all():
    path = os.getcwd()
    filesList = []
    for root, dirs, files in os.walk(path, topdown=True):
        filesList += [(f, root) for f in files if not inRoot(root)]

    setup(
        name = "gjk2",
        ext_modules=[
        Extension("geometry.gjk2",
                  sources=["./geometry/gjk2.pyx", "./geometry/gjk/gjkHPN.c"],
                  include_dirs = ['.', numpy.get_include()],
                  language="c")],
        cmdclass={"build_ext": build_ext})
    setup(
        name = "ik2",
        ext_modules=[
        Extension("geometry.pr2.ik2",
                  sources=["./geometry/pr2/pr2InvKin2.pyx",
                           "./geometry/pr2/IK/ikfastApr18Left.cpp",
                           "./geometry/pr2/IK/ikfastApr18Right.cpp"],
                  include_dirs = ['.', numpy.get_include()],
                  language="c++")],
        cmdclass={"build_ext": build_ext})

    num = 0
    for (fname, path) in filesList:
        if fname.find("gjk_def2") >= 0:
          continue
        if fname.find("pr2InvKin2") >= 0:
          continue
        i = fname.find(".pyx")
        if i != -1 and fname.find("~") == -1:
            convert(fname, path, i)
            num += 1
    return num

def clear_all():
    path = os.getcwd()
    filesList = []
    for root, dirs, files in os.walk(path, topdown=True):
        filesList += [(f, root) for f in files if not inRoot(root)]

    filesToRemove = []
    for (fname, path) in filesList:
        i3 = fname.find("gjkHPN.c") # gjkHPN.c is needed to compile gjk...
        if any(fname.endswith(ext) for ext in [".c", ".so"]) and i3 == -1:
            filePath = path + '/' + fname
            filesToRemove.append(filePath)
            print 'Going to remove: ', filePath

    ans = raw_input('Are you sure you want to remove the above files? [yes | no]: ')
    if ans.lower() == 'yes':
        for file in filesToRemove:
            os.remove(file)
        return len(filesToRemove)
    return 0

def main():
    args = ['build_ext', '--inplace']
    if sys.argv[-len(args):] != args:
      sys.argv += args
      print 'Added', args, 'to args'

    filename = raw_input("Input filename [all | clear | *.pyx]: ")
    i = filename.find(".pyx")

    num = 0
    if filename.lower() == "all":
        num += build_all()
    elif filename.lower() == "clear":
        num += clear_all()
    elif i != -1:
        path = getPath(filename)
        convert(filename, path, i)
        num += 1
    else:
        print "Error: Invalid filetype (name.pyx)"
    print num, "files processed"

if __name__ == '__main__':
    main()

#python setup.py build_ext --inplace
# all
#Caelan Garrett
