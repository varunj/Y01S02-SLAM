'''
testing
python run_oxford_new.py 2014-07-14-15-42-55/

final
python run_oxford_new.py 2014-12-12-10-45-15/ > a.txt
python run_oxford_new.py 2015-05-19-14-06-38/ > b.txt
python run_oxford_new.py 2015-05-22-11-14-30/ > c.txt
'''
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import warnings
warnings.filterwarnings('ignore')
plt.switch_backend('agg')
import glob
from colour_demosaicing import demosaicing_CFA_Bayer_bilinear as demosaic
from PIL import Image
from camera_model import CameraModel
import csv
import sys



route           = sys.argv[1]
dataImgLeft     = '/mnt/efs/temp/oxford-robotcar/' + route + 'stereo/left/'
dataImgRight    = '/mnt/efs/temp/oxford-robotcar/' + route + 'stereo/right/'
outputPath      = '/mnt/efs/temp/outs/oxford-robotcar-new/' + route
refFile         = outputPath + 'pts_history_file.txt'
model           = CameraModel('/mnt/efs/temp/oxford-robotcar/robotcar-dataset-sdk-3.1/models/', '/mnt/efs/temp/oxford-robotcar/' + route + 'stereo/left/')



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



def write_txt(fn, verts, colors, i):
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
    print(verts.shape)
    out_colors = colors.copy()
    verts = verts.reshape(-1, 3)
    out = np.zeros((verts.shape[0], verts.shape[1]+2))
    for ix, eachPt in enumerate(verts):
        out[ix][0] = i
        out[ix][1], out[ix][2], out[ix][3] = eachPt[0], eachPt[1], eachPt[2]
        out[ix][4] = 0.2126*out_colors[ix][0] + 0.7152*out_colors[ix][1] + 0.0722*out_colors[ix][2]

    f=open(fn,'ab')
    np.savetxt(f, out, delimiter=' ', fmt='%f')
    f.close()




def doStuff(fileImgLeft, fileImgRight, savePath, i):
    img_left_color = Image.open(fileImgLeft)
    img_right_color = Image.open(fileImgRight)
    img_left_color = demosaic(img_left_color, 'gbrg')
    img_right_color = demosaic(img_right_color, 'gbrg')
    img_left_color = model.undistort(img_left_color)
    img_right_color = model.undistort(img_right_color)
    img_left_color = np.array(img_left_color).astype(np.uint8)
    img_right_color = np.array(img_right_color).astype(np.uint8)

    imgLeftBW = cv2.blur(cv2.cvtColor(img_left_color, cv2.COLOR_RGB2GRAY),(5,5))
    imgRightBW = cv2.blur(cv2.cvtColor(img_right_color, cv2.COLOR_RGB2GRAY),(5,5))


    # step 1 - calculate disparity map
    stereo = cv2.StereoBM_create(numDisparities=96, blockSize=11)
    disparity = stereo.compute(imgLeftBW, imgRightBW)
    disparityImg = disparity.copy()


    # step 2 - calculate the 3D point cloud with XYZRGB information
    cam1 = [[983.0440,  0.0,        643.646973],
            [0.0,       983.0440,   493.378998],
            [0.0,       0.0,        1.0     ]]
    cam1 = np.array(cam1)
    cam2 = cam1

    Tmat = np.array([0.24, 0., 0.])
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

    # reflected_pts = np.matmul(out_points, reflect_matrix)
    # projected_img,_ = cv2.projectPoints(reflected_pts, np.identity(3), np.array([0., 0., 0.]), cam2[:3,:3], np.array([0., 0., 0., 0.]))
    # projected_img = projected_img.reshape(-1, 2)
    # imgBackProjected = np.zeros(img_left_color.shape, 'uint8')
    # img_colors = img_right_color[mask][idx].reshape(-1,3)

    # for i, pt in enumerate(projected_img):
    #     pt_x = int(pt[0])
    #     pt_y = int(pt[1])
    #     if pt_x > 0 and pt_y > 0:
    #         col = (int(img_colors[i, 2]), int(img_colors[i, 1]), int(img_colors[i, 0]))
    #         cv2.circle(imgBackProjected, (pt_x, pt_y), 1, col)

    # write_ply(outputPath + 'pointCloud/' + fileName.split('.')[0] + '.ply', out_points, out_colors)
    # plt.imsave(outputPath + 'imageLeftBW/' + fileName, imgLeftBW, cmap='gray')
    # plt.imsave(outputPath + 'imageRightBW/' + fileName, imgRightBW, cmap='gray')
    # plt.imsave(outputPath + 'disparityMap/' + fileName, disparityImg, cmap='hsv')
    # plt.imsave(outputPath + 'backProjection/' + fileName, cv2.cvtColor(imgBackProjected, cv2.COLOR_RGB2BGR))

    write_txt(outputPath + 'pts_history_file_my.txt', out_points, out_colors, i)
    # print('done: ', fileName)



if __name__ == '__main__':
    with open(refFile, newline='') as f:
        reader = csv.reader(f, delimiter=' ')
        data = list(reader)

    incomingIDs = [int(x[0]) for x in data]
    incomingIDs = list(set(incomingIDs))


    for i, eachFile in enumerate(sorted(glob.glob(dataImgLeft + '*.png'))):
        if (i in incomingIDs):          
            fileName        = eachFile.split('/')[-1]
            fileImgLeft     = dataImgLeft + fileName
            fileImgRight    = dataImgRight + fileName

            try:
                doStuff(fileImgLeft, fileImgRight, fileName, i)
                print('done: ', i)
            except Exception as e:
                print('error: ', i)
                print(e)
                pass
