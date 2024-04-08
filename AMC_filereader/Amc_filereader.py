import os

def read_amc_rotation(file_path, part_name, data_index):
    rotations = []
    frame_number = 1
    with open(file_path, 'r') as file:
        for line in file:
            if part_name in line:
                rotation_values = line.split()
                rotation = float(rotation_values[data_index])
                rotations.append((frame_number, rotation))
                frame_number += 1
    return rotations

def write_to_csv(output_file, data):
    with open(output_file, 'w') as file:
        file.write("Frame,Rotation\n")
        for frame, rotation in data:
            file.write(f'{frame},{rotation}\n')

# set file and parameter
file_path = '../IDE-starter/VS2017/interpolate/131_04-dance_lq20.amc'
# name of body part
part_name = 'lfemur'
# which argument, for most of situation x is 0, y is 1, z is 2
data_index = 0

rotations = read_amc_rotation(file_path, part_name, data_index)

file_name_without_extension = os.path.splitext(os.path.basename(file_path))[0]
output_file = file_name_without_extension + '_' + part_name + '_' + str(data_index) + '.csv'
write_to_csv(output_file, rotations)
