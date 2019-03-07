from setuptools import setup


def readme():
    with open('README.rst') as f:
        return f.read()


setup(name='modbus-sim',
      version='0.1.0',
      description='A Modbus slave device simulator',
      url='https://github.com/gbrucepayne/modbus-sim',
      author='G Bruce Payne',
      author_email='gbrucepayne@hotmail.com',
      license='MIT',
      packages=['modbus_sim'],
      install_requires=[
            'pymodbus>=2.1',
            'twisted',
            'pyserial>=3.4',
            'requests',
      ],
      include_package_data=True,
      zip_safe=False)
