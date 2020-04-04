'''
https://medium.com/analytics-vidhya/depth-sensing-and-3d-reconstruction-512ed121aa60

ffmpeg -framerate 10 -pattern_type glob -i 'imageLeftBW/*.png' -vf 'scale=-2:min(1080\,trunc(ih/2)*2)' -c:v libx264 -pix_fmt yuv420p imageLeftBW.mp4
ffmpeg -framerate 10 -pattern_type glob -i 'imageRightBW/*.png' -vf 'scale=-2:min(1080\,trunc(ih/2)*2)' -c:v libx264 -pix_fmt yuv420p imageRightBW.mp4
ffmpeg -framerate 10 -pattern_type glob -i 'disparityMap/*.png' -vf 'scale=-2:min(1080\,trunc(ih/2)*2)' -c:v libx264 -pix_fmt yuv420p disparityMap.mp4
ffmpeg -framerate 10 -pattern_type glob -i 'backProjection/*.png' -vf 'scale=-2:min(1080\,trunc(ih/2)*2)' -c:v libx264 -pix_fmt yuv420p backProjection.mp4

ffmpeg -i imageLeftBW.mp4 -i imageRightBW.mp4 -i disparityMap.mp4 -i backProjection.mp4 -filter_complex "[0:v][1:v]hstack=inputs=2[top];[2:v][3:v]hstack=inputs=2[bottom];[top][bottom]vstack=inputs=2[v]" -map "[v]" output.mp4
'''
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import warnings
warnings.filterwarnings('ignore')
plt.switch_backend('agg')
import glob



dataImgLeft     = '../data/data_scene_flow/testing/image_2/'
dataImgRight    = '../data/data_scene_flow/testing/image_3/'
dataCalib       = '../data/data_scene_flow_calib/testing/calib_cam_to_cam/'



def write_ply(fn, verts, colors):
    ply_header = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    '''
    out_colors = colors.copy()
    verts = verts.reshape(-1, 3)
    verts = np.hstack([verts, out_colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')



def doStuff(fileImgLeft, fileImgRight, fileCalib, savePath):
    img_left_color = cv2.imread(fileImgLeft)
    img_right_color = cv2.imread(fileImgRight)
    imgLeftBW = cv2.blur(cv2.cvtColor(img_left_color, cv2.COLOR_RGB2GRAY),(5,5))
    imgRightBW = cv2.blur(cv2.cvtColor(img_right_color, cv2.COLOR_RGB2GRAY),(5,5))


    # step 1 - calculate disparity map
    stereo = cv2.StereoBM_create(numDisparities=96, blockSize=11)
    disparity = stereo.compute(imgLeftBW, imgRightBW)
    disparityImg = disparity.copy()


    # step 2 - calculate the 3D point cloud with XYZRGB information
    matrix_type_1 = 'P_rect_02'
    matrix_type_2 = 'P_rect_03'
    with open(fileCalib, 'r') as f:
        fin = f.readlines()
        for line in fin:
            if line[:9] == matrix_type_1:
                calib_matrix_1 = np.array(line[10:].strip().split(' ')).astype('float32').reshape(3,-1)
            elif line[:9] == matrix_type_2:
                calib_matrix_2 = np.array(line[10:].strip().split(' ')).astype('float32').reshape(3,-1)


    cam1 = calib_matrix_1[:,:3] # left image - P2
    cam2 = calib_matrix_2[:,:3] # right image - P3
    Tmat = np.array([0.54, 0., 0.])
    rev_proj_matrix = np.zeros((4,4))
    cv2.stereoRectify(cameraMatrix1 = cam1,cameraMatrix2 = cam2, \
                      distCoeffs1 = 0, distCoeffs2 = 0, \
                      imageSize = img_left_color.shape[:2], \
                      R = np.identity(3), T = Tmat, \
                      R1 = None, R2 = None, \
                      P1 =  None, P2 =  None, Q = rev_proj_matrix);
    points = cv2.reprojectImageTo3D(disparityImg, rev_proj_matrix)


    #reflect on x axis
    reflect_matrix = np.identity(3)
    reflect_matrix[0] *= -1
    points = np.matmul(points,reflect_matrix)

    #extract colors from image
    colors = cv2.cvtColor(img_left_color, cv2.COLOR_BGR2RGB)

    #filter by min disparity
    mask = disparityImg > disparityImg.min()
    out_points = points[mask]
    out_colors = colors[mask]

    #filter by dimension
    idx = np.fabs(out_points[:,0]) < 4.5
    out_points = out_points[idx]
    out_colors = out_colors.reshape(-1, 3)
    out_colors = out_colors[idx]

    reflected_pts = np.matmul(out_points, reflect_matrix)
    projected_img,_ = cv2.projectPoints(reflected_pts, np.identity(3), np.array([0., 0., 0.]), cam2[:3,:3], np.array([0., 0., 0., 0.]))
    projected_img = projected_img.reshape(-1, 2)
    imgBackProjected = np.zeros(img_left_color.shape, 'uint8')
    img_colors = img_right_color[mask][idx].reshape(-1,3)

    for i, pt in enumerate(projected_img):
        pt_x = int(pt[0])
        pt_y = int(pt[1])
        if pt_x > 0 and pt_y > 0:
            col = (int(img_colors[i, 2]), int(img_colors[i, 1]), int(img_colors[i, 0]))
            cv2.circle(imgBackProjected, (pt_x, pt_y), 1, col)

    write_ply('outs/pointCloud/' + fileName.split('.')[0] + '.ply', out_points, out_colors)
    plt.imsave('outs/imageLeftBW/' + fileName, imgLeftBW, cmap='gray')
    plt.imsave('outs/imageRightBW/' + fileName, imgRightBW, cmap='gray')
    plt.imsave('outs/disparityMap/' + fileName, disparityImg, cmap='hsv')
    plt.imsave('outs/backProjection/' + fileName, cv2.cvtColor(imgBackProjected, cv2.COLOR_RGB2BGR))
    print('done: ', fileName)



if __name__ == '__main__':
    
    for eachFile in sorted(glob.glob(dataImgLeft + '*.png')):
        fileName        = eachFile.split('/')[-1]
        fileImgLeft     = dataImgLeft + fileName
        fileImgRight    = dataImgRight + fileName
        fileCalib       = dataCalib + fileName[:6] + '.txt'
        doStuff(fileImgLeft, fileImgRight, fileCalib, fileName)

