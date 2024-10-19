## SS3DM Codebase

[Our paper]() describes our data export based on CARLA. For more information, see the official CARLA documentation: [Official Documentation](https://carla.readthedocs.io/en/latest/).

## Quick Start

1. Install and configure CARLA. For installing CARLA's Python library, refer to the [official documentation](https://carla.readthedocs.io/en/latest/). We recommend using Python 3.7.

2. Install the prerequisite libraries used in the code.

   ```bash
   pip3 install -r requirements.txt
   ```

3. Open the CARLA simulator, choose one of the cities, load it, and then click "Run" in the top right corner. Here, we will use Town02 as an example.

   ![fig1](imgs\fig1.png)

4. In the `path.txt` of our codebase, select a path (you can also define your own path), and modify `export/export_car_manager.py` with the `route_points` and `transform`. Set `route_points` to the coordinates of all points along the path, and set `transform` to the coordinates and rotation corresponding to the starting point. For example, if we choose the `Town_02 300frames` path, the modified code snippet should look like this:

   ```python
   route_points = [
       carla.Location(x=4610.736328 / 100, y=28746.634766 / 100, z=22.363468 / 100),
       carla.Location(x=4589.283203 / 100, y=20432.091797 / 100, z=22.353996 / 100)
   ]
   
   transform = carla.Transform(carla.Location(x=4610.736328 / 100, y=28746.634766 / 100, z=22.363468 / 100), carla.Rotation(pitch=0.025818, yaw=-89.975494, roll=0.000184))
   ```

5. Modify the `frame_count` in `export/export.py` to the desired number of frames, which in this example is 300 frames. This variable is located at line 114 of the code.

6. Run `export/export.py` and wait for the export to complete. The data will be exported to the directory where `export.py` is located.

7. (Optional) If you need to convert the data to Streetsurf format, copy `parse2streetsurf.py` from `data_processing` to the exported data's `data/` directory and run it to convert the data format.

## Default Organization of Exported Data

By default, the exported data in `data/` will contain 8 folders, each storing the following types of data:

- The `depths` folder contains depth data captured by the depth camera, stored in `exr` format. For specific parameters of the depth camera, please refer to our paper. Each frame's data is aligned with the RGB camera.
- The `images` folder contains image data captured by the RGB camera.
- The `insseg` folder contains instance segmentation data from the instance annotation camera, where each color in the image represents a different entity. The same entity will have the same color across different frames from different cameras. Each frame's data is aligned with the RGB camera.
- The `lidars` folder contains point cloud data from the LiDAR, stored in `npz` format. For specific parameters, please refer to our paper or the `lidar2ply.py` script.
- The `poses` folder contains the positional information for each frame's camera, including the rotation matrix from Camera 5 (front camera) to the world coordinate system and the rotation matrices from different cameras to the front camera.
- The `semseg` folder contains image data from the semantic segmentation camera; refer to the CARLA official documentation for the semantic meanings corresponding to colors. Each frame's data is aligned with the RGB camera.
- The `vehicles` and `walkers` folders contain information about other vehicles and pedestrians. This section of the information is not yet fully processed and will be supplemented in future updates.

## Explanation of Code Functionality

- ### Coordinate System Conversion

  Our dataset may involve multiple coordinate systems; therefore, we provide coordinate transformation scripts located in the `coord_transform` folder.

  `fbx2obj_axis.py` is used to convert the coordinate system of the fbx files exported from CARLA (after being converted to `obj` files) to the world coordinate system we use.

- ### Data Export

  The scripts related to data export are located in the `export` folder.

  We provide `path.txt`, which records all routes included in our dataset (Note: the frame count may not be accurate, as we made minor adjustments based on the route length);

  We provide `export.py` for exporting data. When running it, please ensure that you have correctly configured CARLA, and it will export the data to the `data/` folder in the same directory.

  If you need to modify the route and frame count in `export.py`, please edit `export_car_manager.py` and change `frame_count` in `export.py`.

  We provide `fbx2obj.py`, located in the `misc` folder. This script converts the `fbx` files exported from CARLA to `obj` files. Please note that CARLA does not export materials when exporting `fbx` files, so you will need to handle the materials to some extent.

- ### Data Processing

  To facilitate the processing of the dataset, we provide data processing scripts located in the `misc` folder.

  `parse2streetsurf.py` is used to convert data exported from CARLA into training data formatted like Streetsurf.

  Currently, it has two modes: when `convert_flag` is `False`, place it in the `data` folder and run it to convert the data in the current folder;

  When `convert_flag` is `True`, it can recursively convert all data in the current subfolder. Please note that we have not debugged this functionality, so we do not recommend enabling this option.

- ### Data Visualization

  We also provide data visualization scripts to help visualize some of the data, located in the `misc` folder.

  `lidar2ply.py` is used to convert point clouds stored in `npz` format in our dataset to `ply` format for easier viewing;

  `semseg2png.py` is used to convert instance segmentation data stored in `npz` format in our dataset to `png` format for visualization.