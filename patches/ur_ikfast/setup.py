from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from setuptools import find_packages

# scipy-openblas32 ships libscipy_openblas.lib / .dll. ur_ikfast's
# generated code calls unprefixed LAPACK symbols; lapack_stubs.cpp
# forwards them to the scipy_* exports, so we link against scipy-openblas32.
try:
    import scipy_openblas32
    _blas_lib_dir = scipy_openblas32.get_lib_dir()
    _blas_lib = scipy_openblas32.get_library()  # "libscipy_openblas"
    _lib_args = dict(library_dirs=[_blas_lib_dir], libraries=[_blas_lib])
except ImportError:
    _lib_args = {}

setup(name='ur_ikfast',
      version='0.1.0',
      license='MIT License',
      long_description=open('README.md').read(),
      packages=find_packages(),
      ext_modules=[Extension("ur3e_ikfast",
                             ["ur3e/ur3e_ikfast.pyx",
                              "ur3e/ikfast_wrapper.cpp",
                              "ur3e/lapack_stubs.cpp"],
                             language="c++",
                             **_lib_args)],
      cmdclass={'build_ext': build_ext})
