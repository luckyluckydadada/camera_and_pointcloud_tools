# camera_and_pointcloud_tools

## sudo apt install pcl-tools
官方tools：https://pcl.readthedocs.io/projects/tutorials/en/latest/walkthrough.html#binaries
```
ls /usr/bin/pcl_*
pcl_add_gaussian_noise                 pcl_mesh2pcd                                pcl_openni_ii_normal_estimation                pcl_plane_projection
pcl_boundary_estimation                pcl_mesh_sampling                           pcl_openni_image                               pcl_ply2obj
pcl_cluster_extraction                 pcl_mls_smoothing                           pcl_openni_klt                                 pcl_ply2pcd
pcl_compute_cloud_error                pcl_modeler                                 pcl_openni_mls_smoothing                       pcl_ply2ply
pcl_compute_hausdorff                  pcl_morph                                   pcl_openni_mobile_server                       pcl_ply2raw
pcl_compute_hull                       pcl_multiscale_feature_persistence_example  pcl_openni_octree_compression                  pcl_ply2vtk
pcl_concatenate_points_pcd             pcl_ndt2d                                   pcl_openni_organized_compression               pcl_plyheader
pcl_converter                          pcl_ndt3d                                   pcl_openni_organized_edge_detection            pcl_png2pcd
pcl_convert_pcd_ascii_binary           pcl_ni_agast                                pcl_openni_organized_multi_plane_segmentation  pcl_point_cloud_editor
pcl_convolve                           pcl_ni_brisk                                pcl_openni_passthrough                         
pcl_poisson_reconstruction
pcl_crf_segmentation                   pcl_ni_linemod                              pcl_openni_pcd_recorder                        
pcl_ppf_object_recognition
pcl_crop_to_hull                       pcl_ni_susan                                pcl_openni_planar_convex_hull                  
pcl_progressive_morphological_filter
pcl_demean_cloud                       pcl_ni_trajkovic                            pcl_openni_planar_segmentation                 
pcl_pyramid_surface_matching
pcl_dinast_grabber                     pcl_nn_classification_example               pcl_openni_save_image                          pcl_radius_filter
pcl_elch                               pcl_normal_estimation                       pcl_openni_shift_to_depth_conversion           
pcl_registration_visualizer
pcl_extract_feature                    pcl_obj2pcd                                 pcl_openni_tracking                            
pcl_sac_segmentation_plane
pcl_face_trainer                       pcl_obj2ply                                 pcl_openni_uniform_sampling                    pcl_spin_estimation
pcl_fast_bilateral_filter              pcl_obj2vtk                                 pcl_openni_viewer                              
pcl_statistical_multiscale_interest_region_extraction_example
pcl_feature_matching                   pcl_obj_rec_ransac_accepted_hypotheses      pcl_openni_voxel_grid                          
pcl_stereo_ground_segmentation
pcl_fpfh_estimation                    pcl_obj_rec_ransac_hash_table               pcl_organized_pcd_to_png                       pcl_surfel_smoothing_test
pcl_fs_face_detector                   pcl_obj_rec_ransac_model_opps               pcl_organized_segmentation_demo                pcl_test_search_speed
pcl_generate                           pcl_obj_rec_ransac_orr_octree               pcl_outlier_removal                            pcl_tiff2pcd
pcl_gp3_surface                        pcl_obj_rec_ransac_orr_octree_zprojection   pcl_outofcore_print                            pcl_timed_trigger_test
pcl_grabcut_2d                         pcl_obj_rec_ransac_result                   pcl_outofcore_process                          
pcl_train_linemod_template
pcl_grid_min                           pcl_obj_rec_ransac_scene_opps               pcl_outofcore_viewer                           
pcl_train_unary_classifier
pcl_ground_based_rgbd_people_detector  pcl_octree_viewer                           pcl_passthrough_filter                         
pcl_transform_from_viewpoint
pcl_hdl_grabber                        pcl_offline_integration                     pcl_pcd2ply                                    pcl_transform_point_cloud
pcl_hdl_viewer_simple                  pcl_oni2pcd                                 pcl_pcd2png                                    
pcl_unary_classifier_segment
pcl_icp                                pcl_oni_viewer                              pcl_pcd2vtk                                    pcl_uniform_sampling
pcl_icp2d                              pcl_openni2_viewer                          pcl_pcd_change_viewpoint                       pcl_vfh_estimation
pcl_image_grabber_saver                pcl_openni_3d_concave_hull                  pcl_pcd_convert_NaN_nan                        pcl_viewer
pcl_image_grabber_viewer               pcl_openni_3d_convex_hull                   pcl_pcd_grabber_viewer                         pcl_virtual_scanner
pcl_in_hand_scanner                    pcl_openni_boundary_estimation              pcl_pcd_image_viewer                           pcl_vlp_viewer
pcl_linemod_detection                  pcl_openni_change_viewer                    pcl_pcd_introduce_nan                          pcl_voxel_grid
pcl_local_max                          pcl_openni_face_detector                    pcl_pcd_organized_edge_detection               
pcl_voxel_grid_occlusion_estimation
pcl_lum                                pcl_openni_fast_mesh                        pcl_pcd_organized_multi_plane_segmentation     pcl_vtk2obj
pcl_manual_registration                pcl_openni_feature_persistence              pcl_pcd_select_object_plane                    pcl_vtk2pcd
pcl_marching_cubes_reconstruction      pcl_openni_grabber_depth_example            pcl_pcd_video_player                           pcl_vtk2ply
pcl_match_linemod_template             pcl_openni_grabber_example                  pcl_pclzf2pcd                                  pcl_xyz2pcd
```
### pcl_viewer
```
一种快速可视化 PCD（点云数据）文件的方法。
语法为： pcl_viewer <file_name 1..N>.<pcd or vtk> <options>，其中选项为：
-bc r,g,b 前景色 如 255,255,255
-fc r,g,b 前景色
-ps X = 点大小 (1..64)
-opaque X = 渲染点云不透明度 (0..1)
-ax n = 启用 XYZ 轴的屏幕显示并将它们缩放到 n
-ax_pos X,Y,Z = 如果轴被启用，设置它们在空间中的 X,Y,Z 位置（默认 0,0,0）
-cam (*) = 使用给定的相机设置作为初始视图
(*) [裁剪范围/焦点/位置/ViewUp/距离/Y 视场/窗口大小/窗口位置] 或使用包含相同信息的 <filename.cam>。
-multiview 0/1 = 启用/禁用自动多视口渲染（默认禁用）
-normals 0/X = 禁用/启用将每个 Xth 点的表面法线显示为线（默认禁用） -normals_scale X = 将法线单位矢量大小调整为 X（默认为 0.02）
-pc 0/X = 禁用/启用将每个 Xth 点的主曲率显示为线（默认禁用） -pc_scale X = 将主曲率向量大小调整为 X（默认 0.02）
（注意：对于多个 .pcd 文件，提供多个 -{fc,ps,opaque} 参数；它们会自动分配给正确的文件）
```
### pcd_convert_NaN_nan
```
将“NaN”值转换为“nan”值。（注意：从 PCL 1.0.1 版开始，NaN 的字符串表示为“nan”。）
用法示例：
pcl_pcd_convert_NaN_nan input.pcd output.pcd
```
### convert_pcd_ascii_binary
```
将 PCD（点云数据）文件从 ASCII 转换为二进制，反之亦然。
用法示例：
pcl_convert_pcd_ascii_binary <file_in.pcd> <file_out.pcd> 0/1/2 (ascii/binary/binary_compressed) [precision (ASCII)]
```
### concatenate_points_pcd
```
将两个或多个 PCD（点云数据）文件的点连接成一个 PCD 文件。
用法示例：
pcl_concatenate_points_pcd <filename 1..N.pcd>
（注意：生成的 PCD 文件将是“output.pcd”）
```
### pcd2vtk
```
将 PCD（点云数据）文件转换为VTK 格式。
用法示例：
pcl_pcd2vtk input.pcd output.vtk
```
### pcd2ply
```
将 PCD（点云数据）文件转换为PLY 格式。
用法示例：
pcl_pcd2ply input.pcd output.ply
```
### mesh2pcd
```
使用光线追踪操作将 CAD 模型转换为 PCD（点云数据）文件。
pcl_mesh2pcd input.{ply,obj} output.pcd <options>，其中选项为：
-level X = 镶嵌球体级别（默认值：2）
-resolution X = 以角度增量表示的球体分辨率（默认值：100 度）
-leaf_size X = VoxelGrid 的 XYZ 叶大小 - 用于数据缩减（默认值：0.010000 m）
```
### octree_viewer
```
允许八叉树的可视化
pcl_octree_viewer <file_name.pcd> <octree resolution>
用法示例：
./octree_viewer ../../test/bunny.pcd 0.02
```


## pcl-viewer
### build
```
cd pcl-viewer
mkdir build
cd build
cmake ..
make
```
### run
viewer
```
./viewer
./color-viewer
```

## calibrate
### build
```
cd calibrate
mkdir build
cd build
cmake ..
make
```
### run
```
./demo
```

## kitti_fusion_pointcloud_and_image
### build
```
cd kitti_fusion_pointcloud_and_image
mkdir build
cd build
cmake ..
make
```
### run
```
./fusion yy.pcd zz.png calib.txt
```

## pcl trans
### build
```
cd pcl-trans
mkdir build
cd build
cmake ..
make
```
### run
kitti2pcd
```
./kittibin2pcd 000000.bin 000000.pcd
python kitti2pcd.py
```
kitti_to_rosbag
```
https://github.com/ethz-asl/kitti_to_rosbag
```
pcdbin2pcdascii_pcdascii2pcdbin Usage:
```
./pcdbin2pcdascii_pcdascii2pcdbin in.pcd out.pcd bin2ascii
./pcdbin2pcdascii_pcdascii2pcdbin in.pcd out.pcd ascii2bin
./pcdbin2pcdascii_pcdascii2pcdbin in.pcd out.pcd compress
```
## kitti_native_evaluation
https://github.com/asharakeh/kitti_native_evaluation

