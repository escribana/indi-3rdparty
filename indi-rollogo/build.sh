#!/bin/bash

# untar source files into ./indi/3rdparty/indi-rollogo
#    readme.txt \
#    build.sh \
#    CMakeLists.txt \
#    config.h.cmake \
#    indi_rollogo.xml.cmake \
#    rollogo.cpp \
#    rollogo.h    

# Create a 3rdparty directory for source files, & install them if needed
# Eaxample:
# mkdir -p /wrk/dev/indi/3rdparty/indi-rollogo
# cd /wrk/dev/indi/3rdparty/rollogo
# tar -xzf /wrk/dev/bu/rolloffino/<even|odd>/rollogo.tar.gz

# Create a clean build directory & set default there
mkdir -p /wrk/dev/indi-build/3rdparty/indi-rollogo
cd /wrk/dev/indi-build/3rdparty/indi-rollogo
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Debug /wrk/dev/indi/3rdparty/indi-rollogo
echo ""
echo "Default directory: " `pwd`
echo ""
echo "Issue: cd /wrk/dev/indi-build/3rdparty/indi-rollogo"
echo "Default directory: " `pwd`
echo "Issue: sudo make install"

# Compare with entries for the rolloff simulator
#
# /usr/share/kstars/indidrivers.xml
#       Standard device label, driver name & executable name
#       For example some telescopes (label) use a common driver
#       Created by KStars build. Might need to manually edit this.

# /usr/share/indi/indi_<rollogo>.xml
#       The additional devices from 3rdparty drivers installed
#       Might need to manually delete this if also in above list

