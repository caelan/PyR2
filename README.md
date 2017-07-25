# amber
A simple infrastructure for robot motion planning.

# Build #

The all cython files (.pyx) are built during the installation. If for some reason they need to be rebuilt (if you change a .pyx or .pxd) run the following command from /amber:

```
python setup.py build_ext --inplace
```

When prompted for a .pyx you can enter the .pyx you wish to build, *clear* to clear all compiled files, or *all* to build all files.
