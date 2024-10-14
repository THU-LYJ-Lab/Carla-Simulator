'''
Put this under "data" folder
This will convert current output data format into streetsurf data format
'''

import os
import pickle
import numpy as np
import json
import cv2
from tqdm import *

# DO NOT set this to True
convert_flag = False

# How many frames do the data have
N_FRAMES = 150

# Which number do the data start
FRAME_START_NUM = 2375

# Name your scene
SCENE_NAME = 'TEST_SCENE'

# Load a printed matrix from a certain path
def load_matrix(path):

	mat = np.array([])

	with open(path, 'r') as f:
		lines = f.readlines()

		for line in lines:
			words = line.split()

			line_arr = []

			for word in words:
				word = word.replace('[', '').replace(']', '')

				try:
					word = float(word)
					line_arr.append(word)

				except:
					pass

			if line_arr:
				if not mat.shape[0]:
					mat = np.array(line_arr)

				else:
					mat = np.row_stack((mat, np.array(line_arr)))

	return mat


def solve_single(directory, datafolder, scene_id):

	# Make necessary directories
	if not os.path.exists(directory):
		os.makedirs(directory)

	depth_directory = os.path.join(directory, 'depth_gts')
	image_directory = os.path.join(directory, 'images')
	lidar_directory = os.path.join(directory, 'lidars')
	semseg_directory = os.path.join(directory, 'masks')

	old_image_directory = os.path.join(datafolder, 'images')
	old_lidar_directory = os.path.join(datafolder, 'lidars')
	old_depth_directory = os.path.join(datafolder, 'depths')
	old_semseg_directory = os.path.join(datafolder, 'semseg')

	if not os.path.exists(image_directory):
		os.makedirs(image_directory)

	if not os.path.exists(lidar_directory):
		os.makedirs(lidar_directory)

	# Create observer dictionary file

	obsev_dic = {}

	## Parse RGB Cameras
	## 0 - Backright
	## 1 - Backleft
	## 2 - Back
	## 3 - Frontright
	## 4 - Frontleft
	## 5 - Front

	camera_folder_name = ['back_right', 'back_left', 'back', 'front_right', 'front_left', 'front']
	camera_folder_name_cap = [string.upper() for string in camera_folder_name]

	## Load intr for cameras, all the intrs are the same

	intr_mat = load_matrix(os.path.join(datafolder, 'calib', 'rgb_intrinsics.txt'))

	for i in tqdm(range(6)):
		cam_dic = {}

		cam_dic['id'] = 'camera_' + camera_folder_name_cap[i]
		cam_dic['class_name'] = 'Camera'
		cam_dic['num_frames'] = N_FRAMES

		cam_data = {}

		hw = np.zeros([N_FRAMES, 2])
		intr = np.zeros([N_FRAMES, 3, 3])
		c2w = np.zeros([N_FRAMES, 4, 4])
		distortion = np.zeros([N_FRAMES, 4])
		sensor_v2w = np.zeros([N_FRAMES, 4, 4])
		c2v = np.zeros([N_FRAMES, 4, 4])

		cam2front = load_matrix(os.path.join(datafolder, 'calib', 'cams', 'rgb_cam2cam_%s.txt' % str(i)))
		cam2front = np.row_stack((cam2front, np.array([0, 0, 0, 1])))

		for j in tqdm(range(N_FRAMES)):
			frame_id = FRAME_START_NUM + j

			# Save the image file into images
			old_image = os.path.join(old_image_directory, camera_folder_name[i], '%.6d.jpg' % frame_id)
			if not os.path.exists(os.path.join(image_directory, 'camera_' + camera_folder_name_cap[i])):
				os.makedirs(os.path.join(image_directory, 'camera_' + camera_folder_name_cap[i]))
			new_image = os.path.join(image_directory, 'camera_' + camera_folder_name_cap[i], '%.8d.jpg' % j)
			open(new_image, 'wb').write(open(old_image, 'rb').read())

			# Fill in metadata
			hw[j] = np.array([1080, 1920])
			intr[j] = intr_mat

			front2world = load_matrix(os.path.join(datafolder, 'poses', 'rgb_cam2world_5_%d.txt' % (j + 1)))
			front2world = np.row_stack((front2world, np.array([0, 0, 0, 1])))
			
			# c2w needs to be in OpenCV coordinate system, so this matrix is required
			c2w_mat = np.array([
				[0, 0, 1, 0],
				[1, 0, 0, 0],
				[0, -1, 0, 0],
				[0, 0, 0, 1]
			])
			c2w[j] = front2world @ cam2front @ c2w_mat
			c2v[j] = np.linalg.inv(c2w_mat) @ cam2front @ c2w_mat
			sensor_v2w[j] = front2world @ c2w_mat

		# Fill in cam_data
		cam_data['hw'] = hw
		cam_data['intr'] = intr
		cam_data['c2w'] = c2w
		cam_data['distortion'] = distortion
		cam_data['c2v'] = c2v
		cam_data['sensor_v2w'] = sensor_v2w
		cam_data['global_frame_ind'] = np.array(range(N_FRAMES))
		cam_data['timestamp'] = np.array(range(FRAME_START_NUM, FRAME_START_NUM + N_FRAMES))

		cam_dic['data'] = cam_data

		obsev_dic['camera_' + camera_folder_name_cap[i]] = cam_dic

	## Parse Depth Cameras

	depth_folder_name = ['depth_back_right', 'depth_back_left', 'depth_back', 'depth_front_right', 'depth_front_left', 'depth_front']

	for i in tqdm(range(6)):
		for j in tqdm(range(N_FRAMES)):
			frame_id = FRAME_START_NUM + j

			# Load the 3 channel depth image
			image = cv2.imread(os.path.join(old_depth_directory, depth_folder_name[i], '%.6d.exr' % frame_id))

			# Convert the image into single channel image
			depth_mat = np.array([256 * 256, 256, 1]) / (256 * 256 * 256 - 1) * 65535
			depth_image = np.dot(image, depth_mat).astype(np.uint16)

			# Save the image file into images
			if not os.path.exists(os.path.join(depth_directory, 'camera_' + camera_folder_name_cap[i])):
				os.makedirs(os.path.join(depth_directory, 'camera_' + camera_folder_name_cap[i]))
			cv2.imwrite(os.path.join(depth_directory, 'camera_' + camera_folder_name_cap[i], '%.8d.png' % j), depth_image)

	## Parse Semsegs

	semseg_folder_name = ['back_right', 'back_left', 'back', 'front_right', 'front_left', 'front']

	for i in tqdm(range(6)):
		for j in tqdm(range(N_FRAMES)):
			frame_id = FRAME_START_NUM + j

			# Save the semseg file into masks
			old_semseg = os.path.join(old_semseg_directory, semseg_folder_name[i], 'semseg_frame_%d.npz' % frame_id)
			if not os.path.exists(os.path.join(semseg_directory, 'camera_' + camera_folder_name_cap[i])):
				os.makedirs(os.path.join(semseg_directory, 'camera_' + camera_folder_name_cap[i]))
			new_semseg = os.path.join(semseg_directory, 'camera_' + camera_folder_name_cap[i], '%.8d.npz' % j)
			open(new_semseg, 'wb').write(open(old_semseg, 'rb').read())

	## Parse LiDARs

	lidar_folder_name = ['lidar_front', 'lidar_left', 'lidar_rear', 'lidar_right', 'lidar_top']
	lidar_folder_name_cap = ['FRONT', 'LEFT', 'REAR', 'RIGHT', 'TOP']

	for i in tqdm(range(5)):
		lidar_dic = {}

		lidar_dic['id'] = 'lidar_' + lidar_folder_name_cap[i]
		lidar_dic['class_name'] = 'RaysLidar'
		lidar_dic['num_frames'] = N_FRAMES

		l2v = np.zeros([N_FRAMES, 4, 4])

		for j in tqdm(range(N_FRAMES)):
			frame_id = FRAME_START_NUM + j

			# Save the image file into images
			old_lidar = os.path.join(old_lidar_directory, lidar_folder_name[i], 'lidar_frame_%d.npz' % frame_id)
			if not os.path.exists(os.path.join(lidar_directory, 'lidar_' + lidar_folder_name_cap[i])):
				os.makedirs(os.path.join(lidar_directory, 'lidar_' + lidar_folder_name_cap[i]))
			new_lidar = os.path.join(lidar_directory, 'lidar_' + lidar_folder_name_cap[i], '%.8d.npz' % j)
			open(new_lidar, 'wb').write(open(old_lidar, 'rb').read())

			front2world = load_matrix(os.path.join(datafolder, 'poses', 'rgb_cam2world_5_%d.txt' % (j + 1)))
			front2world = np.row_stack((front2world, np.array([0, 0, 0, 1])))
			
			# c2w needs to be in OpenCV coordinate system, so this matrix is required
			c2w_mat = np.array([
				[0, 0, 1, 0],
				[1, 0, 0, 0],
				[0, -1, 0, 0],
				[0, 0, 0, 1]
			])
			l2v[j] = np.linalg.inv(front2world @ c2w_mat)

		lidar_dic['data'] = dict(
			l2v = l2v,
			timestamp = np.array(range(FRAME_START_NUM, FRAME_START_NUM + N_FRAMES)), 
			global_frame_ind = np.array(range(N_FRAMES))
		)

		obsev_dic['lidar_' + lidar_folder_name_cap[i]] = lidar_dic

	## Ego Vehicle

	ego_dic = dict(
		class_name = 'EgoVehicle', 
		num_frames = N_FRAMES, 
		data = dict(
			v2w = obsev_dic['camera_FRONT']['data']['c2w'], 
			timestamp = np.array(range(FRAME_START_NUM, FRAME_START_NUM + N_FRAMES)), 
			global_frame_ind = np.array(range(N_FRAMES))
		)
	)
	obsev_dic['ego_car'] = ego_dic

	# Create metadata dict

	metas = {}

	metas['num_frames'] = N_FRAMES
	metas['world_offset'] = np.array([0, 0, 0])
	metas['up_vec'] = '+z'

	# Create scenario dict file

	scenario_dict = {}

	scenario_dict['observers'] = obsev_dic
	scenario_dict['objects'] = {}
	scenario_dict['scene_id'] = scene_id
	scenario_dict['metas'] = metas

	with open(os.path.join(directory, 'scenario.pt'), 'wb') as w:
		pickle.dump(scenario_dict, w)

	# Create readable json

	with open(os.path.join(directory, 'scenario.txt'), 'w') as w:
		w.write(str(scenario_dict))


def main():

	global N_FRAMES, FRAME_START_NUM

	if not convert_flag:
		directory = os.path.join(os.getcwd(), 'streetsurf_data')
		solve_single(directory, os.getcwd(), SCENE_NAME)
	else:
		for dirpath, dirnames, filenames in os.walk(os.getcwd()):
			for name in dirnames:
				if name.isnumeric():
					print('Parsing %s' % (dirpath + '/' + name))

					datafolder = os.path.join(dirpath, name)
					N_FRAMES = int(name)
					cam_dir = os.path.join(dirpath, name, 'images', 'front')
					min_id = 10**10

					for _, __, filenames2 in os.walk(cam_dir):
						for name2 in filenames2:
							name_num = name2.split('.')[0]
							if name_num.isnumeric():
								min_id = min(min_id, int(name_num))

					if min_id < 10**10:
						FRAME_START_NUM = min_id
						print('N_FRAMES: %d, FRAME_START_NUM: %d' % (N_FRAMES, FRAME_START_NUM))
						directory = os.path.join(dirpath, name + '_streetsurf')
						solve_single(directory, datafolder, dirpath.split('/')[-1] + '_' + name)



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
