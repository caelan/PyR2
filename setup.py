#!/usr/bin/env python2

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import sys
import os
import numpy

EXCLUDE_DIRS = ['.git', 'build'] # Directories to exclude when searching for cython files (.pyx)
CLEAN_EXTENSIONS = [".c", ".so"]
KEEP_FILES = ["gjkHPN.c"] # gjkHPN.c is needed to compile gjk.

########################################

def get_files():
    for root, dirs, files in os.walk(os.getcwd(), topdown=True):
        reduxPath = root.replace(os.getcwd(), '')
        if not any(exclude_dir in reduxPath for exclude_dir in EXCLUDE_DIRS):
            for f in files:
                yield (f, root)

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

########################################

def build_all():
    setup(
        name = "gjk2",
        ext_modules=[
        Extension("PyR2.gjk2",
                  sources=["./PyR2/gjk2.pyx", "./PyR2/gjk/gjkHPN.c"],
                  include_dirs = ['.', numpy.get_include()],
                  language="c")],
        cmdclass={"build_ext": build_ext})
    setup(
        name = "ik2",
        ext_modules=[
        Extension("PyR2.pr2.ik2",
                  sources=["./PyR2/pr2/pr2InvKin2.pyx",
                           "./PyR2/pr2/IK/ikfastApr18Left.cpp",
                           "./PyR2/pr2/IK/ikfastApr18Right.cpp"],
                  include_dirs = ['.', numpy.get_include()],
                  language="c++")],
        cmdclass={"build_ext": build_ext})

    for fname, path in get_files():
        if any(name in fname for name in ("gjk_def2", "pr2InvKin2")):
            continue
        i = fname.find(".pyx")
        if i != -1 and fname.find("~") == -1:
            convert(fname, path, i)

########################################

def clear_all():
    for fname, path in get_files():
        if any(fname.endswith(ext) for ext in CLEAN_EXTENSIONS) and \
              not any(keep in fname for keep in KEEP_FILES):
            filePath = path + '/' + fname
            os.remove(filePath)
            print 'Cleaned', filePath

########################################

HELP = 'python setup.py [build | clean]'

def main():
    if len(sys.argv) != 2:
      print HELP
      return

    if sys.argv[1] == "build":
        sys.argv = sys.argv[:1] + ['build_ext', '--inplace']
        build_all()
    elif sys.argv[1] == "clean":
        clear_all()
    else:
        print HELP

if __name__ == '__main__':
    main()