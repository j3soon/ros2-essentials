#! /bin/bash
#build and upload to pYPi
rm -rf dist
rm -rf build
rm -rf *.egg-info
python3 setup.py sdist bdist_wheel
python3 -m twine upload dist/*
#to install: pip install  hello-robot-stretch-urdf
#to uninstall: pip uninstall hello-robot-stretch-urdf
