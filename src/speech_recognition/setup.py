from setuptools import find_packages, setup

package_name = 'speech_recognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'faster-whisper',
        'silero-vad',
        'sounddevice',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Matthew Cromer',
    maintainer_email='matthew.h.cromer@gmail.com',
    description='Local speech recognition using Whisper',
    license='MIT',
    entry_points={
        'console_scripts': [
            'speech_node = speech_recognition.speech_node:main',
        ],
    },
)
