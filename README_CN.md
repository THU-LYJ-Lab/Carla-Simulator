## SS3DM 代码库

[我们的论文]()，我们基于 CARLA 进行数据导出，CARLA 的官方文档：[官方文档](http://carla.org/)

## 快速上手

1. 安装并配置 CARLA，安装 CARLA 的 python 库，请参见 CARLA 的[官方文档](http://carla.org/)。我们推荐您使用 python 3.7

2. 安装代码使用的前置库

   ```
   pip3 install -r requirements.txt
   ```

3. 打开 CARLA 模拟器，选择其中一座城市，打开它，随后点击右上角的运行。这里我们以 Town02 为例。

   ![fig1](imgs\fig1.png)

4. 在我们代码库的 `path.txt` 中选择一条路径（您也可以自行制定路径），修改 `export/export_car_manager.py` 中的 `route_points` 与 `transform`。将 `route_points` 修改为路径上所有点的坐标，将 `transform` 设置为起点对应的坐标与旋转。例如我们选择了 `Town_02 300frames` 路径，修改后的代码片段应该如下：

   ```python
   		route_points = [
               carla.Location(x=4610.736328 / 100,y=28746.634766 / 100,z=22.363468 / 100),
               carla.Location(x=4589.283203 / 100,y=20432.091797 / 100,z=22.353996 / 100)
           ]
           
           transform = carla.Transform(carla.Location(x=4610.736328 / 100,y=28746.634766 / 100,z=22.363468 / 100), carla.Rotation(pitch=0.025818,yaw=-89.975494,roll=0.000184))
           
   ```

5. 修改 `export/export.py` 中的 `frame_count` 至想要导出的帧数，在这个例子中为 300 帧。该变量默认位于代码的 114 行。

6. 运行 `export/export.py`，等待导出完成。数据会被导出至 `export.py` 所在的路径下。

7. （可选）如果需要将数据处理成 Streetsurf 格式，则将 `data_processing` 中的 `parse2streetsurf.py` 复制到导出数据的 `data/` 目录下，运行即可转换数据格式。

## 导出数据的默认组织模式

默认情况下，导出的数据 `data/` 会有 8 个文件夹，每个文件夹以及内部存放的数据含义如下：

- `depths` 文件夹存放深度相机拍摄的深度数据，数据以 `exr` 格式存储，深度相机的具体参数请参见我们的论文。每一帧的数据与 RGB 相机对齐。
- `images` 文件夹存放 RGB 相机拍摄的图像数据。
- `insseg` 文件夹存放实体标注相机的实体标注数据，图中的每种不同颜色代表一个不同的实体。同一实体在不同相机的不同帧中的颜色是相同的。每一帧的数据与 RGB 相机对齐。
- `lidars` 文件夹存放 LiDAR 的点云数据，数据以 `npz` 格式存储，具体参数请参见我们的论文或是参考 `lidar2ply.py` 脚本。
- `poses` 文件夹存放每一帧相机的位置信息，包含 `5` 号相机（前置相机）到世界坐标系的旋转矩阵以及不同相机到前置相机的旋转矩阵。
- `semseg` 文件夹存放语义分割相机的图像数据，颜色对应的语义请参考 CARLA 官方文档。每一帧的数据与 RGB 相机对齐。
- `vehicles` 与 `walkers` 存放其他车辆与行人的信息。该部分的信息处理目前尚未完善，将在后续的更新中补充。

## 代码具体功能讲解

- ### 坐标系转换

  我们的数据集可能涉及多个坐标系，因此在我们的代码中提供了坐标系转换脚本，这些脚本位于 `coord_transform` 文件夹下。

  `fbx2obj_axis.py` 用于将 carla 导出的 fbx 文件（转成 `obj` 文件后）的坐标系转换到我们使用的世界坐标系。

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