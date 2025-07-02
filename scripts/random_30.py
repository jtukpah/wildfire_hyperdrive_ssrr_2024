#!/usr/bin/env python3

import os
import glob
import random
import shutil

bag_file_name = 'olin_collection_05_15_23_parcel_b_v4'

'''
Completed items
bag_file_name = 'olin_collection_05_16_23_parcel_b_v2'
bag_file_name = 'olin_collection_05_16_23_parcel_b'
bag_file_name = 'olin_collection_05_16_23_campus'
bag_file_name = 'olin_collection_05_15_23_parcel_b'
bag_file_name = 'olin_collection_05_15_23_parcel_b_v2'
bag_file_name = 'olin_collection_05_15_23_parcel_b_v3'


---

'''


source_directory = "/home/river/catkin_ws/src/hyper_drive/ImageData/" + bag_file_name

count_num = 0

# Destination directory to move the selected images
destination_directory_imec = "/home/river/catkin_ws/src/hyper_drive/ImageData/labeled_data_500_v2/IMEC"
destination_directory_rgb = "/home/river/catkin_ws/src/hyper_drive/ImageData/labeled_data_500_v2/RGB"
destination_directory_spectra = "/home/river/catkin_ws/src/hyper_drive/ImageData/labeled_data_500_v2/SPECTRA"
destination_directory_ximea = "/home/river/catkin_ws/src/hyper_drive/ImageData/labeled_data_500_v2/XIMEA"

# Number of images to select

num_images = 72

# Get a list of all files in the source directory per type
imec_files = [file for file in os.listdir(source_directory + '/imec')]
rgb_files = [file for file in os.listdir(source_directory + '/RGB')]
spectra_files = [file for file in os.listdir(source_directory + '/spectra')]
ximea_files = [file for file in os.listdir(source_directory + '/ximea')]

# Set a random seed for reproducibility
random.seed(123)

# Select the same set of indices for each file list
selected_indices = random.sample(range(len(spectra_files)), num_images)
print(selected_indices)
# Move the selected images to the destination directory
for index in selected_indices:
    # Get the filenames at the selected indices
    #  imec_filename = imec_files[index]
    rgb_filename = rgb_files[index]
    # NOW WE NEED TO FIND THE VALUE OF THE ACTUAL INDEX
    use_idx = rgb_filename.split('_')[-1].split('.')[0]
    spectra_filename = f'spectra_{use_idx}.npy'
    ximea_filename = f'Cube_{use_idx}.npy'
    imec_filename = f'Cube_{use_idx}.npy'
    # # Generate new filenames for the images
    new_imec_filename = f'{bag_file_name}_{count_num}'
    new_rgb_filename = f'{bag_file_name}_{count_num}'
    new_spectra_filename = f'{bag_file_name}_{count_num}'
    new_ximea_filename = f'{bag_file_name}_{count_num}'

    # # Build the source and destination paths for each image
    imec_source_path = os.path.join(source_directory, 'imec', imec_filename)
    rgb_source_path = os.path.join(source_directory, 'RGB', rgb_filename)
    spectra_source_path = os.path.join(source_directory, 'spectra', spectra_filename)
    ximea_source_path = os.path.join(source_directory, 'ximea', ximea_filename)

    #  print(imec_source_path)
    # print(rgb_source_path)
    imec_destination_path = os.path.join(destination_directory_imec, new_imec_filename)
    rgb_destination_path = os.path.join(destination_directory_rgb, new_rgb_filename)
    spectra_destination_path = os.path.join(destination_directory_spectra, new_spectra_filename)
    ximea_destination_path = os.path.join(destination_directory_ximea, new_ximea_filename)

    # Move the images to the destination directory
    shutil.copy(imec_source_path, imec_destination_path + os.path.splitext(imec_source_path)[1])
    shutil.copy(rgb_source_path, rgb_destination_path + os.path.splitext(rgb_source_path)[1])
    shutil.copy(spectra_source_path, spectra_destination_path + os.path.splitext(spectra_source_path)[1])
    shutil.copy(ximea_source_path, ximea_destination_path + os.path.splitext(ximea_source_path)[1])

    count_num += 1