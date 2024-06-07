from setuptools import setup
import versioneer

setup(
    name="starpas",
    version=versioneer.get_version(),
    cmdclass=versioneer.get_cmdclass(),
    author="Jonas Witthuhn",
    author_email="witthuhn@tropos.de",
    license="OSI Approved :: GNU General Public License v3 (GPLv3)",
    packages=["starpas"],
    package_dir={"": "src"},
    package_data={"starpas": ["share/*.json"]},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'starpas = starpas.click:cli',
        ],
    },
    python_requires=">=3.10",
    install_requires=[
        "numpy",
        "scipy",
        "pandas",
        "xarray",
        "netcdf4",
        "matplotlib",
        "pip",
        "jstyleson",
        "Click",
        "toolz",
    ],
    extras_require={
        "nbs": [
            "jupyter",
            "nbdev",
            "nbformat",
        ],
        "docs": [
            "sphinx",
            "myst-parser",
            "myst-nb",
        ],
    },
)
