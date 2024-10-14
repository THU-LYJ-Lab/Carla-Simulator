## Carla Simulator

[Official Document](http://carla.org/)

## Usage

All the external Python libraries required for our dataset are listed in requirements.txt. Please note that to use all the code in our dataset, you must install and configure Carla. The link can be found above.

When using our code, we recommend using Python 3.7 as using other versions of Python may lead to compatibility issues.

```
pip3 install -r requirements.txt
```

- ### Coordinate Transformations

  Our dataset may involve multiple coordinate systems, so we provide coordinate conversion scripts in our code, located in the `coord_transform` folder.

  `fbx2obj_axis.py` is used to convert the coordinate system of FBX files exported from CARLA to our world coordinate system.

  `left2right.py` and `right2left.py` are used to convert between left-handed and right-handed coordinate systems.

  `carla2obj.py` and `obj2carla.py` are used to convert between our world coordinate system and the OBJ coordinate system exported from CARLA.

- ### Data Export

  Scripts related to data export are located in the `export` folder.

  We provide `path.txt`, which records all the routes included in our dataset (note: the frame count may not be accurate, as we made minor adjustments to the frame count based on the length of the routes).

  We provide `export.py` for data export. Make sure CARLA is configured correctly when running this script, and it will export data to the `data/` folder in the same directory.

  If you need to modify the routes and frame count in `export.py`, please edit `export_car_managet.py` and adjust the `frame_count` in `export.py`.

  We provide `fbx2obj.py`, located in the `misc` folder, which converts FBX files exported from CARLA to OBJ files. Please note that when exporting FBX files from CARLA, materials are not exported, so you will need to process the materials accordingly.

- ### Data Processing

  To facilitate the processing of the dataset, we provide a data processing script located in the `misc` folder.

  `parse2streetsurf.py` is used to convert data exported from CARLA into training data, following the same format used by Streetsurf.

  It currently has two modes. When `convert_flag` is set to `False`, place it in the `data` folder and run it to convert the data in the current folder.

  When `convert_flag` is set to `True`, it can recursively convert all data in the current subfolders. Please note that this functionality has not been tested, so we do not recommend enabling this option.

- ### Data Visualization

  We also provide data visualization scripts to facilitate the visualization of certain data. These scripts are located in the `misc` folder.

  `lidar2ply.py` converts point cloud data saved in `npz` format from our dataset into `ply` format for easier viewing.

  `semseg2png.py` converts semantic segmentation data saved in `npz` format from our dataset into `png` format for visualization.

我们数据集所需要的所有外部 pip 库已经位于 `requirements.txt` 中。请注意，为了正常使用我们数据集中的所有代码，你必须安装并正确配置 carla，carla 的链接见上方。

在使用我们代码时，我们推荐您使用 python 3.7，使用其他版本的 python 可能会带来不兼容的问题。

```
pip3 install -r requirements.txt
```

- ### 坐标系转换

  我们的数据集可能涉及多个坐标系，因此在我们的代码中提供了坐标系转换脚本，这些脚本位于 `coord_transform` 文件夹下。

  `fbx2obj_axis.py` 用于将 carla 导出的 fbx 的坐标系转换到我们使用的世界坐标系；

  `left2right.py` 与 `right2left.py` 用于互相转换左手系与右手系；

  `carla2obj.py` 与 `obj2carla.py` 用于互相转换我们使用的世界坐标系与 carla 导出的 obj 的坐标系。

- ### 数据导出

  数据导出相关脚本位于 `export` 文件夹下。

  我们提供了 `path.txt`，记录了我们数据集中包含的所有路线（注：帧数可能不准确，因为根据路线长度短我们对帧数做了少量调整）；

  我们提供 `export.py`，用于对数据进行导出，运行它时请确保你正确配置了 carla，它会将数据导出至同目录下的 `data/` 中。

  如果需要修改 `export.py` 的路线和帧数，请修改 `export_car_managet.py` 并修改 `export.py` 中的 `frame_count`。

  我们提供 `fbx2obj.py`，该脚本位于 `misc` 文件夹下。其功能为将 carla 导出的 `fbx` 文件转换为 `obj` 文件。需要注意，carla 导出 `fbx` 文件时不会导出材质，因此你需要对材质进行一定程度的处理。

- ### 数据处理

  为了方便对数据集进行处理，我们提供了数据处理脚本，这个脚本位于 `misc` 文件夹下。

  其中 `parse2streetsurf.py` 用于将从 carla 导出的数据转化成训练用数据，训练数据使用的格式与 Streetsurf 相同。

  目前其拥有两个模式，在 `convert_flag` 为 `False` 时，将其放置在 `data` 文件夹下并运行即可令其转化当前文件夹中的数据；

  在 `convert_flag` 为 `True` 时，其可以递归转化当前子文件夹下的所有数据。请注意，我们并没有针对该功能进行调试，因此我们不推荐将该选项开启。

- ### 数据可视化

  我们还提供了数据可视化脚本，便于将部分数据可视化，这些脚本位于 `misc` 文件夹下。

  `lidar2ply.py` 用于将我们数据集中以 `npz` 格式保存的点云转换为 `ply`，便于进行查看；

  `semseg2png.py` 用于将我们数据集中以 `npz` 格式保存的实体分割数据转换为 `png` 便于进行可视化。