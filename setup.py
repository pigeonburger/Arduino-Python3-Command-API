import setuptools

with open("README.md", "r") as fh:
  long_description = fh.read()

setuptools.setup(
  name="arduino-python3",
  version="0.7",
  install_requires=['pyserial'],
  author="Pigeonburger",
  author_email="pigeonburger@pigeonburger.xyz",
  description="A light-weight Python library that provides a serial \
  bridge for communicating with Arduino microcontroller boards. Extended to work with Python 3",
  long_description=long_description,
  long_description_content_type="text/markdown",
  url='https://github.com/pigeonburger/Arduino-Python3-Command-API',
  packages=['Arduino'],
  license='MIT',
)
