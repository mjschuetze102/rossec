import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rossec",
    version="0.0a3",
    author="Michael Schuetze",
    author_email="mjschuetze102@gmail.com",
    description="A package to add encryption to ROS 1 projects",
    license="MIT License",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/mjschuetze102/ROS-Encryption",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2",
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
    ],
    install_requires=[
        'pycryptodome',
    ],
    python_requires='>=2.7, !=3.0.*, !=3.1*, !=3.2*, !=3.3*, !=3.4*, !=3.5*',
)

