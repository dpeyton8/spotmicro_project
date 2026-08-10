from setuptools import setup

setup(
    name='spot_micro_kinematics',
    version='1.0',
    description='A useful module',
    author='Mike Romanko',
    author_email='foomail@foo.com',
    packages=['spot_micro_kinematics', 'spot_micro_kinematics.utilities'],
    package_dir={'spot_micro_kinematics': '.'},
    install_requires=['numpy', 'matplotlib'],
)
