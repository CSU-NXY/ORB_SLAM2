seq=09
./Examples/RGB-D/rgbd_kitti \
Vocabulary/ORBvoc.txt \
Examples/RGB-D/KITTI.yaml \
/home/dl/Downloads/kitti_odom_test/sequences/${seq} \
pred_depths/${seq}_resized/predictions \
pred_depths/${seq}_resized/uncertainties


#gt_poses/09_transQuat.txt \
#gt_poses/09_transQuat.txt

# 注意这里的uncertainty路径是错的，之后记得改掉
