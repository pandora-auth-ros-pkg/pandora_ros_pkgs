#!/usr/bin/env python
"""Checks whether an image annotation in an annotation file refers to a non
existing image in a dataset.
"""
import os
import sys
import shutil

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

    print "Type the absolute path to the dataset folder:"
    print "e.g. /home/user/data"
    print "This path should contain 2 folders with name 'Training_Images' "\
          "and 'Test_Images'"
    imagesPath = raw_input()

    trainingImagesPath = os.path.join(imagesPath, 'Training_Images')
    testImagesPath = os.path.join(imagesPath, 'Test_Images')

    print "Type the absolute path to the annotations folder:"
    print "e.g. /home/user/data"
    annotationsPath = raw_input()

    print "Type the name of the training annotations file:"
    print "e.g. training_annotations.txt"
    trainingAnnotationsFileName = raw_input()

    print "Type the name of the test annotations file:"
    print "e.g. test_annotations.txt"
    testAnnotationsFileName = raw_input()

    trainingSetFileName = os.path.join(annotationsPath,
                                       trainingAnnotationsFileName)
    testSetFileName = os.path.join(annotationsPath,
                                   testAnnotationsFileName)

    print "The new annotation files will have the same name as the ones you "\
          "typed with the prefix 'new_'"
    newTrainingAnnotationsFileName = "new_" + trainingAnnotationsFileName
    newTestAnnotationsFileName = "new_" + testAnnotationsFileName
    newTrainingSetFileName = os.path.join(annotationsPath,
                                          newTrainingAnnotationsFileName)
    newTestSetFileName = os.path.join(annotationsPath,
                                      newTestAnnotationsFileName)

    trainingSetData = open(trainingSetFileName, 'r')
    testSetData = open(testSetFileName, 'r')

    newTrainingSetData = open(newTrainingSetFileName, 'w')
    newTestSetData = open(newTestSetFileName, 'w')

    nameIndex = 1
    for line in trainingSetData:
        imageName, imageData = line.split(',', 1)
        nameConst, nameNumber = imageName.split('frame')
        nameNumber, fileExt = nameNumber.split('.png')

        newNameNumber = str(nameIndex).zfill(5)
        nameIndex += 1
        newImageName = 'frame' + newNameNumber + '.png'
        newData = newImageName + ',' + imageData

        newTrainingSetData.write(newData)
        if imageName in os.listdir(trainingImagesPath):
            print imageName, " was renamed to ", newImageName
            shutil.move(trainingImagesPath + "/" + imageName,
                        trainingImagesPath + "/" + newImageName)

    trainingSetData.close()

    for line in testSetData:
        imageName, imageData = line.split(',', 1)
        nameConst, nameNumber = imageName.split('frame')
        nameNumber, fileExt = nameNumber.split('.png')

        newNameNumber = str(nameIndex).zfill(5)
        nameIndex += 1
        newImageName = 'frame' + newNameNumber + '.png'
        newData = newImageName + ',' + imageData

        newTestSetData.write(newData)
        if imageName in os.listdir(testImagesPath):
            print imageName, " was renamed to ", newImageName
            shutil.move(testImagesPath + "/" + imageName,
                        testImagesPath + "/" + newImageName)

    testSetData.close()
