#!/usr/bin/env python
"""Transforms image annotations from specific bounding boxes to full frame
bounding boxes
"""

import collections

# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Kofinas Miltiadis"
__maintainer__ = "Kofinas Miltiadis"
__email__ = "mkofinas@gmail.com"


if __name__ == "__main__":
    print "This script transforms annotations to full frame bounding boxes."
    print "The size of all the images is supposed to be 640 x 480."
    print "Type the absolute path for the file you want to read"
    print "e.g. /home/user/foo/bar/annotations.txt"
    filePath = raw_input()
    fileData = open(filePath, "r")
    print "Type the absolute path for the file you want to write to"
    print "e.g. /home/user/foo/bar/new_annotations.txt"
    newFilePath = raw_input()
    newFileData = open(newFilePath, "w")
    newXTLP = 0
    newYTLP = 0
    newXBRP = 639
    newYBRP = 439
    annotationsDict = collections.defaultdict(list)
    for line in fileData:
        imageName, classAttribute, xTLP, yTLP, xBRP, yBRP = line.split(",")
        annotationsDict[imageName].append((classAttribute, newXTLP, newYTLP,
                                           newXBRP, newYBRP))

    fileData.close()

    for dictKey, dictValues in annotationsDict.iteritems():
        newFileData.write(dictKey)
        newFileData.write(",")
        newFileData.write(str(dictValues[0][0]))
        newFileData.write(",")
        newFileData.write(str(dictValues[0][1]))
        newFileData.write(",")
        newFileData.write(str(dictValues[0][2]))
        newFileData.write(",")
        newFileData.write(str(dictValues[0][3]))
        newFileData.write(",")
        newFileData.write(str(dictValues[0][4]))
        newFileData.write("\n")

    newFileData.close()
    print "Done!"
