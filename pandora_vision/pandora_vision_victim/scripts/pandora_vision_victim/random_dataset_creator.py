#!/usr/bin/env python
"""Provides functionalities to create a random subset of an already existing
dataset.
"""
import os
import shutil
import random
import readline

from collections import defaultdict

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


class RandomDatasetCreator(object):
    """Creates a random sub-set of a dataset.

    This class is used to create a random sub-set of an already existing
    dataset. The original dataset contains class labeled images for a
    two-class classification problem. The sub-set chosen has about the same
    percentage of positive and negative images as the original one.
    """
    def __init__(self, src_annotations_path,
                 dst_annotations_path, desired_dataset_size,
                 save_remaining_annotations=True, positives_percentage=0):
        """Initialize class member variables.

        Args:
            src_annotations_path: The absolute path of the initial annotations
                file.
            dst_annotations_path: The absolute path of the new annotations file.
            desired_dataset_size: Size of the desired dataset.
            save_remaining_annotations: Flag indicating whether the non chosen images
                from the initial dataset will be saved. Default value = False.
            positives_percentage: The percentage of the positives images of the
                new dataset.
        """

        self.annotations_dict = defaultdict(list)
        self.new_annotations_dict = defaultdict(list)
        self.remaining_annotations_dict = defaultdict(list)
        self.positive_image_names = []
        self.negative_image_names = []
        self.new_positive_image_names = []
        self.new_negative_image_names = []
        self.num_positives = 0
        self.num_negatives = 0
        self.desired_dataset_size = int(desired_dataset_size)
        self.positives_percentage = float(positives_percentage)

        self.src_annotations_file_path = src_annotations_path
        self.src_annotations_file_name = os.path.basename(src_annotations_path)

        self.dst_annotations_file_name = os.path.basename(dst_annotations_path)
        self.dst_annotations_path = os.path.dirname(dst_annotations_path)
        self.save_remaining_annotations = save_remaining_annotations

    def read_annotations_file(self):
        """This function reads a file containing the class attributes for each
        image of the original dataset.
        """
        # Check that the file exists.
        if not os.path.isfile(self.src_annotations_file_path):
            print "ERROR : Source annotations file does not exist."
            exit(1)

        # Check that the file extension is appropriate.
        file_extension = ".txt"
        if file_extension not in self.src_annotations_file_name:
            print self.src_annotations_file_name, "is not a proper text file."
            exit(2)
        # Read the annotations file.
        annotations_file = open(os.path.join(self.src_annotations_file_path),
                                "r_u")

        # Ensure that the file was read successfully.
        if (annotations_file is None):
            print "Error reading the file", self.src_annotations_file_path
            exit(3)

        # Read file and save contents in a dictionary.
        for line in annotations_file:
            (frame_name, class_attribute, x_top_left_point, y_top_left_point,
                x_bottom_right_point, y_bottom_right_point) = line.split(",")
            self.annotations_dict[frame_name].append((int(class_attribute),
                                                     int(x_top_left_point),
                                                     int(y_top_left_point),
                                                     int(x_bottom_right_point),
                                                     int(y_bottom_right_point)))

            if int(class_attribute) == 1:
                self.num_positives += 1
                self.positive_image_names.append(frame_name)
            else:
                self.num_negatives += 1
                self.negative_image_names.append(frame_name)

        total_num_images = self.num_positives + self.num_negatives
        if total_num_images < self.desired_dataset_size:
            print "ERROR: Not enough images to create a dataset"
            print "Available Images:", total_num_images
            exit(1)
        if self.positives_percentage <= 0.0:
            self.positives_percentage = (float(self.num_positives) /
                                         total_num_images)
        annotations_file.close()

    def create_annotations_file(self):
        """This function creates the annotations file for the sub-set of the
        dataset that was created.
        """

        # Check that the path exists.
        if not os.path.isdir(self.dst_annotations_path):
            print "ERROR : Incorrect Path for annotations file."
            exit(1)

        # Check that the file extension is appropriate.
        file_extension = ".txt"
        if file_extension not in self.dst_annotations_file_name:
            print self.dst_annotations_file_name, "is not a proper file."
            exit(2)
        # Open the annotations file.
        annotations_file = open(os.path.join(self.dst_annotations_path,
                                             self.dst_annotations_file_name),
                                             "w")

        # Save new annotations in file.
        for (frame_name, dict_values) in self.new_annotations_dict.iteritems():
            annotations_file.write(frame_name)
            annotations_file.write(",")
            annotations_file.write(str(dict_values[0][0][0]))
            annotations_file.write(",")
            annotations_file.write(str(dict_values[0][0][1]))
            annotations_file.write(",")
            annotations_file.write(str(dict_values[0][0][2]))
            annotations_file.write(",")
            annotations_file.write(str(dict_values[0][0][3]))
            annotations_file.write(",")
            annotations_file.write(str(dict_values[0][0][4]))
            annotations_file.write("\n")
        annotations_file.close()

        if (self.save_remaining_annotations):
            remaining_file_name = "remaining_" + self.dst_annotations_file_name
            annotations_file = open(os.path.join(self.dst_annotations_path,
                                    remaining_file_name), "w")

            # Save new annotations in file.
            for (frame_name, dict_values) in self.remaining_annotations_dict.iteritems():

                annotations_file.write(frame_name)
                annotations_file.write(",")
                annotations_file.write(str(dict_values[0][0][0]))
                annotations_file.write(",")
                annotations_file.write(str(dict_values[0][0][1]))
                annotations_file.write(",")
                annotations_file.write(str(dict_values[0][0][2]))
                annotations_file.write(",")
                annotations_file.write(str(dict_values[0][0][3]))
                annotations_file.write(",")
                annotations_file.write(str(dict_values[0][0][4]))
                annotations_file.write("\n")
            annotations_file.close()

    def choose_random_subset(self):
        """This function creates a random sub-set of the original dataset,
        with the same percentage of positive and negative images as the
        original.
        """
        num_positives = int(self.desired_dataset_size *
                            self.positives_percentage)
        num_negatives = int(self.desired_dataset_size - num_positives)

        random_positives_list = random.sample(
                xrange(len(self.positive_image_names)), num_positives)
        random_negatives_list = random.sample(
                xrange(len(self.negative_image_names)), num_negatives)
        # Create random sub-sets for positive and negative images.
        for iii in xrange(num_positives):
            self.new_positive_image_names.append(
                    self.positive_image_names[random_positives_list[iii]])
        for iii in xrange(num_negatives):
            self.new_negative_image_names.append(
                    self.negative_image_names[random_negatives_list[iii]])
        # Create the new annotations dictionary.
        for annotator_key, annotator_values in self.annotations_dict.iteritems():
            if (annotator_key in self.new_positive_image_names or
                annotator_key in self.new_negative_image_names):

                self.new_annotations_dict[annotator_key].append(
                    annotator_values)
            elif self.save_remaining_annotations:
                self.remaining_annotations_dict[annotator_key].append(
                    annotator_values)

    def make_directory(self, folder_path):
        """Make a directory if it doesn't already exist.
        """
        if not os.path.isdir(folder_path):
            print ("Directory " + folder_path + " does not exist and " +
                   "will be created")
            os.mkdir(folder_path)

    def create_neccessary_folders(self, src_image_path, dst_image_path,
                                  copy_rgb_and_depth):
        """Create the folders required in order to copy the new dataset images.
        """
        if not os.path.isdir(src_image_path):
            print "ERROR : Incorrect Path for the source image dataset"
            exit(1)

        src_directories = []
        if copy_rgb_and_depth:
            rgb_src_image_path = os.path.join(src_image_path, "rgb")
            if not os.path.isdir(rgb_src_image_path):
                print "error : incorrect path for the rgb image dataset"
                exit(1)
            depth_src_image_path = os.path.join(src_image_path, "depth")
            if not os.path.isdir(depth_src_image_path):
                print "error : incorrect path for the depth image dataset"
                exit(1)
            src_directories.append(rgb_src_image_path)
            src_directories.append(depth_src_image_path)
        else:
            src_directories.append(src_image_path)

        self.make_directory(dst_image_path)

        dst_directories = []
        if copy_rgb_and_depth:
            rgb_dst_image_path = os.path.join(dst_image_path, "rgb")
            self.make_directory(rgb_dst_image_path)
            depth_dst_image_path = os.path.join(dst_image_path, "depth")
            self.make_directory(depth_dst_image_path)
            dst_directories.append(rgb_dst_image_path)
            dst_directories.append(depth_dst_image_path)
        else:
            dst_directories.append(dst_image_path)

        for dst_directory in dst_directories:
            self.make_directory(os.path.join(dst_directory, "Training_Images"))
            if self.save_remaining_annotations:
                self.make_directory(os.path.join(dst_directory, "Test_Images"))

        return src_directories, dst_directories

    def copy_dataset_images(self, src_image_path, dst_image_path,
                            copy_rgb_and_depth):
        """This function copies the randomly chosen dataset images to a
        specified folder.

        Args:
            src_image_path: The absolute path to the initial dataset, i.e. the
                path to the folder containing the initial dataset images.
            dst_image_path: The absolute path to the new dataset, i.e. the path
                to the folder that will contain the new dataset images.
            copy_rgb_and_depth: Flag indicating whether the initial dataset
                contains both rgb and depth images to be copied.
        """
        src_directories, dst_directories = self.create_neccessary_folders(
                src_image_path, dst_image_path, copy_rgb_and_depth)
        image_types = [".png", ".jpg", ".bmp", ".pgm"]

        for src_directory, dst_directory in zip(src_directories,
                                                dst_directories):
            for file_name in sorted(os.listdir(src_directory)):
                is_image = False
                for image_type in image_types:
                    if image_type in file_name:
                        is_image = True
                        break
                if not is_image:
                    print file_name, "is not an image"
                    continue

                if (file_name in self.new_positive_image_names or
                    file_name in self.new_negative_image_names):

                    shutil.copyfile(os.path.join(src_directory, file_name),
                                    os.path.join(dst_directory,
                                                 "Training_Images",
                                                 file_name))
                elif self.save_remaining_annotations:
                    shutil.copyfile(os.path.join(src_directory, file_name),
                                    os.path.join(dst_directory,
                                                 "Test_Images",
                                                 file_name))

    def create_dataset(self, src_image_path, dst_image_path,
                       copy_rgb_and_depth):
        """This function creates a random dataset that is a subset of another
        dataset.
        """
        self.read_annotations_file()
        self.choose_random_subset()
        self.create_annotations_file()
        self.copy_dataset_images(src_image_path, dst_image_path,
                                 copy_rgb_and_depth)

    def create_new_dataset_annotations(self):
        """This function creates a random dataset's annotations."""
        self.read_annotations_file()
        self.choose_random_subset()
        self.create_annotations_file()

    def show_tree(self, src_images_path, dst_images_path,
                  copy_rgb_and_depth=True):
        """Show the desired format of an already existing dataset and the output
        format of the folders of the new dataset.
        """
        print src_images_path + ":"
        if copy_rgb_and_depth:
            print "|--> rgb:"
            print "|    |--> ..."
            print "|--> depth:"
            print "     |--> ..."
        else:
            print "|--> ..."

        print 30 * "-"
        print dst_images_path + ":"
        if copy_rgb_and_depth:
            print "|--> rgb:"
            print "|    |--> Training_Images:"
            print "|    |    |--> ..."
            if self.save_remaining_annotations:
                print "|    |--> Test_Images:"
                print "|         |--> ..."
            print "|--> depth:"
            print "     |--> Training_Images:"
            print "     |    |--> ..."
            if self.save_remaining_annotations:
                print "     |--> Test_Images:"
                print "          |--> ..."
        else:
            print "|--> Training_Images:"
            print "|    |--> ..."
            if self.save_remaining_annotations:
                print "|--> Test_Images:"
                print "     |--> ..."

def create_dataset():
    """Read neccessary parameters and create a random dataset."""
    # Set the auto completion scheme
    readline.set_completer_delims(" \t")
    readline.parse_and_bind("tab:complete")

    print "Type the absolute path to the initial annotations file:"
    print "e.g. /home/user/baz/annotations.txt"
    src_annotations_path = raw_input("-->")

    print "Type the absolute path to the new annotations file:"
    print "e.g. /home/user/baz/new_annotations.txt"
    dst_annotations_path = raw_input("-->")

    print "Type the desired dataset size"
    desired_dataset_size = raw_input("-->")
    print "Type the desired percentage of the positive images in the new "\
          "dataset size. If you wish to retain the positive/negative ratio, "\
          "press [0] or a negative value."
    positives_percentage = raw_input("-->")


    print "Do you want to save the annotations for the "\
          "remaining images?"
    print "If so, press [y]. Every other key will be considered as a no."
    save_remaining_annotations = raw_input("-->")
    if save_remaining_annotations == "y":
        print "The annotations file for the remaining images will be stored "\
              "in the same directory as the new annotations file and its "\
              "name will have the prefix 'remaining_'."
        save_remaining_annotations = True
    else:
        save_remaining_annotations = False

    print "Would you like to copy the new dataset images? If so, press [y]."
    copy_new_dataset_images = raw_input("-->")
    copy_rgb_and_depth = False

    if copy_new_dataset_images == "y":
        print "Would you like to copy both RGB and Depth images? If so, "\
              "press [y]."
        copy_rgb_and_depth = raw_input("-->")
        if copy_rgb_and_depth == "y":
            copy_rgb_and_depth = True
        else:
            copy_rgb_and_depth = False
        print "Type the absolute path to the initial dataset:"
        print "e.g. /home/user/foo"
        src_image_path = raw_input("-->")
        print "Type the absolute path to the new dataset:"
        dst_image_path = raw_input("-->")

    rand_dataset = RandomDatasetCreator(src_annotations_path,
                                        dst_annotations_path,
                                        desired_dataset_size,
                                        save_remaining_annotations,
                                        positives_percentage)
    print "Creating new Annotations File!"
    rand_dataset.create_new_dataset_annotations()
    if copy_new_dataset_images == "y":
        rand_dataset.show_tree(src_image_path, dst_image_path,
                               copy_rgb_and_depth)
        rand_dataset.copy_dataset_images(src_image_path, dst_image_path,
                                         copy_rgb_and_depth)
    print "Process is finished successfully!"


if __name__ == "__main__":
    create_dataset()
