# Always prefer setuptools over distutils
from setuptools import setup, find_packages
import os

version = '0.0.0'
here = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(here, 'VERSION.txt')) as version_file:
    version = version_file.read().strip()

setup(
    name='bs_module',

    # Versions should comply with PEP440.  For a discussion on single-sourcing
    # the version across setup.py and the project code, see
    # https://packaging.python.org/en/latest/single_source_version.html
    version=version,

    description='Module for bs used.',
    long_description="Module for bs used.",

    # The project's main homepage.
    download_url='',
    url='',

    # Author details
    author='BrightSoul',
    author_email='653538096@qq.com',

    # Choose your license
    license='MIT',

    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        # How mature is this project? Common values are
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        'Development Status :: 4 - Beta',

        # Indicate who your project is intended for
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Embedded Systems',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Mathematics',
        'Topic :: Scientific/Engineering :: Physics',
        'Topic :: Scientific/Engineering :: Visualization',


        # Pick your license as you wish (should match "license" above)
        'License :: OSI Approved :: MIT License',

        # Specify the Python versions you support here. In particular, ensure
        # that you indicate whether you support Python 2, Python 3 or both.
        
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8'
    ],
        
        

    # What does your project relate to?
    keywords=[
        'quaternion', 'math', 'maths', 'physics', 'orientation', 'pose', 'geometry', 'visualisation', 'animation'
    ],

    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    packages=[
        'bs_imu',
        'bs_img',
    ],

    # Alternatively, if you want to distribute just a my_module.py, uncomment
    # this:
    # py_modules=["bs_imu"],

    # List run-time dependencies here.  These will be installed by pip when
    # your project is installed. For an analysis of "install_requires" vs pip's
    # requirements files see:
    # https://packaging.python.org/en/latest/requirements.html
    install_requires=[
        "numpy",
    ],

    # List additional groups of dependencies here (e.g. development
    # dependencies). You can install these using the following syntax,
    # for example:
    # $ pip install -e .[dev,test]
    extras_require={
        'dev': ["mkdocs"],
        'test': ["nose"]
    },

    # If there are data files included in your packages that need to be
    # installed, specify them here.  If using Python 2.6 or less, then these
    # have to be included in MANIFEST.in as well.
    package_data={
    },

    # Although 'package_data' is the preferred approach, in some case you may
    # need to place data files outside of your packages. See:
    # http://docs.python.org/3.4/distutils/setupscript.html#installing-additional-files # noqa
    # In this case, 'data_file' will be installed into '<sys.prefix>/my_data'
    data_files=[],

    # To provide executable scripts, use entry points in preference to the
    # "scripts" keyword. Entry points provide cross-platform support and allow
    # pip to create the appropriate form of executable for the target platform.
    entry_points={},

    # Use nose to discover all tests in the module
    test_suite='nose.collector',

    # Set Nose as a requirement for running tests
    tests_require=['nose'],
)
