seq=01
./Examples/RGB-D/rgbd_kitti \
Vocabulary/ORBvoc.txt \
Examples/RGB-D/KITTI00-02.yaml \
/home/dl/Downloads/kitti_odom_test/sequences/${seq} \
pred_depths/${seq}/predictions \
pred_depths/${seq}/uncertainties \
gt_poses/${seq}_transQuat.txt \
gt_poses/${seq}_transQuat.txt

# 注意这里的uncertainty路径是错的，之后记得改掉
