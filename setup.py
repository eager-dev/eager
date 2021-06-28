import os
from setuptools import setup
from setuptools.command.install import install

class InstallCommand(install):
    user_options = install.user_options + [
        ('packages=', None, 'Specify the packages to install.'),
    ]

    def initialize_options(self):
        install.initialize_options(self)
        self.packages = None

    def finalize_options(self):
        #print('The custom option for install is ', self.custom_option)
        os.system("echo installing EAGER")
        if self.packages is not None:
            package_list = str(self.packages).split()
            available_packages = os.listdir(os.getcwd())
            for package in package_list:
                assert package in available_packages, 'Invalid package {}!'.format(package)
        install.finalize_options(self)

    def run(self):
        global packages
        packages = self.packages
        os.system("echo installing EAGER")
        install.run(self)  # OR: install.do_egg_install(self)

setup(cmdclass={'install': InstallCommand},
      name='eager',
      version='0.0.1',
      description='EAGER: Engine Agnostic Gym Environment for Robotics.',
      url='https://github.com/eager-dev/eager',
      author="Bas van der Heijden, Alexander Keijzer, Jelle Luijkx",
      author_email="j.d.luijkx@tudelft.nl",
      scripts = ['bin/install_eager'],
)