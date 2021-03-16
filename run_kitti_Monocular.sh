seq=09
./Examples/Monocular/mono_kitti \
Vocabulary/ORBvoc.txt \
Examples/Monocular/KITTI04-12.yaml \
/home/dl/Downloads/kitti_odom_test/sequences/${seq} \
gt_poses/${seq}_transQuat.txt \
gt_poses/${seq}_transQuat.txt \
pred_depths/${seq}_resized/uncertainties 

# 注意这里的uncertainty路径是错的，之后记得改掉
