#!/bin/bash

# Complete usage:
#
# The normal directory holds all the pictures of walls obtained physically with
# a camera.
# Create a new directory, with a suitable name (see below) and copy the normal
# images in it.
# By using the batch-color-contrast script for GIMP
# (see http://registry.gimp.org/node/23499), tweak the brightness of the
# images in the new directory, using the same names as the original images.
#
# Eventually, every directory created will hold the same number of images as the
# normal directory, with every image in it tweaked in brightness by the same
# amount.
#
# Declare the names of the directories made below.
# This script should lie in the "walls" directory.


# name counter
counter=0

# directory paths
base_path=$PWD
normal_path="/normal"
plus_100_path="/plus_100"
plus_50_path="/plus_50"
minus_100_path="/minus_100"
minus_50_path="/minus_50"

# images' suffix
suffix=".png"

# normal images path
for f in $(ls $base_path$normal_path); do
  cp $base_path$normal_path/$f $base_path/$counter$suffix
  echo $f
  counter=$(($counter+1))
done

# +50 brightness images path
for f in $(ls $base_path$plus_50_path); do
  cp $base_path$plus_50_path/$f $base_path/$counter$suffix
  echo $f
  counter=$(($counter+1))
done

# +100 brightness images path
for f in $(ls $base_path$plus_100_path); do
  cp $base_path$plus_100_path/$f $base_path/$counter$suffix
  echo $f
  counter=$(($counter+1))
done

# -50 brightness images path
for f in $(ls $base_path$minus_50_path); do
  cp $base_path$minus_50_path/$f $base_path/$counter$suffix
  echo $f
  counter=$(($counter+1))
done

# -100 brightness images path
for f in $(ls $base_path$minus_100_path); do
  cp $base_path$minus_100_path/$f $base_path/$counter$suffix
  echo $f
  counter=$(($counter+1))
done
