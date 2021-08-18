import os
from tqdm import tqdm

def kitti2pcl(in_dir,out_dir):
    files = [d for d in os.listdir(in_dir)]
    isExists=os.path.exists(out_dir)
    if not isExists:
        os.makedirs(out_dir) 
    else:
        print(out_dir+' Have Existed')
    for f in tqdm(files):
        cmd = "./bin2pcd "+in_dir+f+" "+out_dir+f.split('.')[0]+'.pcd'
        os.system(cmd)
		
if __name__ == "__main__": 
    BASE='/home/lucky/d/kitti/'
    TRAIN = BASE+'/training/velodyne/'
    # TEST =  BASE+'/testing/velodyne/'
    
    IN = TRAIN 
    OUT = BASE + 'training/velodyne_pcd/'
    kitti2pcl(IN,OUT)
	
    #IN = TEST + '/velodyne/'
    #OUT = TEST + '/velodyne_pcd/'
    #kitti2pcl(IN,OUT)
    
