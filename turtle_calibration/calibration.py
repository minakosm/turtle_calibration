import numpy as np
import cv2 as cv
import glob
import argparse
import os
import warnings
import pytransform3d.plot_utils as plt_u
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager


#Ignore Deprication Warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
#Termination Criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def parse():
    # Parser function in order to parse arguments through the terminal
    # Needed arguments are: image_dir and image_format. All other arguments have a default value

    parser = argparse.ArgumentParser(prog='calibration', description='Camera Calibration')
    parser.add_argument('--image_dir', type=str, required=True, help='Images directory')
    parser.add_argument('--image_format', type=str, required=True, help='Image format: png/jpg')
    parser.add_argument('--square_size', type=float, default='11', help='Chessboard square size (in cm)')
    parser.add_argument('--width', type=int, default='4', help='Chessboard width')
    parser.add_argument('--height', type=int, default='6', help='Chessboard height')
    parser.add_argument('--intrinsic_calib', type=float, default=0, help='select 1 to include intrinsic calibration')
    arguments = parser.parse_args()
    return arguments

#Save the camera matrix and the distortion after we run the intrinsic calibration
def save_coeffs(mtx, dist, path):
    cv_file = cv.FileStorage((path + 'intrinsic_params') , cv.FILE_STORAGE_WRITE)
    cv_file.write('K', mtx)
    cv_file.write('D', dist)
    cv_file.release()

#Load camera matrix and distortion from existing file
def load_coeffs(path):
    cv_file = cv.FileStorage(path, cv.FileStorage_READ)
    camera_matrix = cv_file.getNode('K').mat()
    distortion_matrix = cv_file.getNode('D').mat()
    return [camera_matrix, distortion_matrix]

#Prepare object points in (0 ,0, 0), (1, 0, 0), (0, 1, 0)... form
def prepare_objp(square_size, width, height):
    objp = np.zeros((height * width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size
    return objp

def draw(img, corners, imgpts):
    corners = corners.astype("int")
    imgpts = imgpts.astype("int")
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (0,0,255), 20)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 20)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (255,0,0), 20)
    return img


#Intrinsic calibration
def calibrate(dirpath, image_format, square_size, width, height):
    print(dirpath, image_format, square_size)

    objp = prepare_objp(square_size, width, height)

    objpoints = []  #3D object points
    imgpoints = []  #2D image points

    images = glob.glob(dirpath + '*.' + image_format)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        #Find chessboard corners
        ret, corners = cv.findChessboardCorners(gray, (width, height), None)

        if ret:
            objpoints.append(objp)

            #Refine chessboard corners in subpixel accuracy
            corners_refined = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners_refined)

            #Draw the corners on the image
            cv.drawChessboardCorners(img, (width, height), corners_refined, ret)
            cv.imshow('img', img)
            cv.waitKey(500)

    cv.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return [ret, mtx, dist, rvecs, tvecs]



#Undistort the images and save the results in directory ~/.../image_dir/results
def undistortion(mtx, dist, path, image_format):
    images = glob.glob(path + '/*.' + image_format)
    i = 1
    directory = (path + 'results')
    os.chdir(directory)
    for img in images:
        img_read = cv.imread(img)
        h, w = img_read.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        dst = cv.undistort(img_read, mtx, dist, None, newcameramtx)

        # crop
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite('calibresult' + str(i) + '.png', dst)
        i = i+1


def rotate_chessboard():
    directionXY = list(map(str ,input("Enter directions of (X,Y) as shown in the image (up/down/left/right): ").strip().split()))[:2]

    if directionXY[0] == 'up':
        if directionXY[1] == 'right':
            rotation = pr.active_matrix_from_intrinsic_euler_xyz([0, np.pi/2, np.pi])
            return rotation
        else:
            raise ValueError("Invalid axis rotation")
    elif directionXY[0] == 'down':
        if directionXY[1] == 'left':
            rotation = pr.active_matrix_from_intrinsic_euler_xyz([0, - np.pi/2, 0])
            return rotation
        else:
            raise ValueError("Invalid axis rotation")
    elif directionXY[0] == 'left':
        if directionXY[1] == 'up':
            rotation = pr.active_matrix_from_intrinsic_euler_xyz([- np.pi/2, 0, - np.pi/2])
            return rotation
        else:
            raise ValueError("Invalid axis rotation")
    elif directionXY[0] == 'right':
        if directionXY[1] == 'down':
            rotation = pr.active_matrix_from_intrinsic_euler_xyz([np.pi/2, 0, np.pi/2])
            return rotation
        else:
            raise ValueError("Invalid axis rotation")
    else:
        raise ValueError("Invalid axis directions")




def manage_transforms(SensorT, chessboard2sensor, ChessboardT):
    # Given the calculated transforms, in addition the translation vector from chessboard2unsprung
    # calculate the wanted Sensor2Unsprung transformation matrix.

    #Take the translation vector chessboard2unsprung as a user input
    tc_t = list(map(np.float32, input("Enter x y z translation parameters from the unsprung to the chessboard (in m): ").strip().split()))[:3]

    Chessboard2Unsprung = pt.transform_from(np.identity(3, np.float32), tc_t)

    tm = TransformManager()

    tm.add_transform("Sensor_turtle", "sensor_opencv", SensorT)
    tm.add_transform("chessboard_opencv", "sensor_opencv", chessboard2sensor)
    tm.add_transform("Chessboard_turtle", "chessboard_opencv", ChessboardT)
    tm.add_transform("Chessboard_turtle", "Unsprung", Chessboard2Unsprung)

    S2T = tm.get_transform("Sensor_turtle", "Unsprung")
    print(S2T)


    plt.figure(figsize=(260,260))

    ax = plt_u.make_3d_axis(3,121,'m')
    tm.plot_frames_in("Sensor_turtle", ax=ax, alpha=0.6)
    ax.view_init(30,20)
    plt.show()


    return S2T




#Calibration in order to find the Camera2Unsprug Transformation Matrix
def external_calib(image_dir, image_format, square_size, width, height):
   mtx, dist = load_coeffs(image_dir + '/intrinsic_params')
   objp = prepare_objp(square_size, width, height)

   #Get the undistorted images
   for fname in glob.glob(image_dir + '/*.' + image_format):
       img = cv.imread(fname)
       gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
       #Find chessboard corners
       ret, corners = cv.findChessboardCorners(gray, (width, height), None)

       if ret:
           corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)


           #Find the rotation and translation vectors
           ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)

           axis = np.float32([[square_size * 3, 0, 0], [0, square_size * 3, 0], [0, 0, square_size * 3]]).reshape(-1, 3)
           imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)

           img = draw(img, corners2, imgpts)

           img = cv.resize(img, (500, 500))
           cv.imshow('img', img)
           cv.waitKey(0)
           cv.destroyAllWindows()

           #Transform the rotation vector to a 3x3 rotation matrix
           rmtx, _ = cv.Rodrigues(rvecs, None, None)
           tvecs.shape = (3,)


           #Calculate translation in meters
           tvecs = tvecs/100

           print(tvecs)
           #Get the transform from chessboard_opencv2sensor_opencv
           c2s = pt.transform_from(rmtx, tvecs)

           #Change the openCV frame convention into ours : x=up, y=left, z=up
           #For the camera sensor: first we rotate the x axis 90deg and then z for 90
           #!!!DESCRIBE OPENCV WITH RESPECT TO TURTLE CONVENTION!!!
           Rs_cv2turtle = pr.active_matrix_from_intrinsic_euler_xyz([np.pi/2, 0, np.pi/2])

           #For the chessboard: insert what the picture dictated
           #!!!DESCRIBE OPENCV WITH RESPECT TO TURTLE CONVENTION!!!
           Rc_cv2turtle = rotate_chessboard()

           #Get transforms from sensor and camera default coordinations to Aristurtle's conventions
           S2s = pt.transform_from(Rs_cv2turtle, 0)  #Camera Sensor
           C2c=  pt.transform_from(Rc_cv2turtle, 0)  #Chessboard

           #Call a transform manager in order to calculate the Sensor2Unsprung transformation
           S2U = manage_transforms(S2s, c2s, C2c)
           return S2U

def main():
    args = parse()
    if(args.intrinsic_calib):
        ret, mtx, dist, rvecs, tvecs = calibrate(args.image_dir, args.image_format, args.square_size, args.width, args.height)

        save_coeffs(mtx, dist, args.image_dir)
        print("Calibration finished. RMS: ", ret)

        #Save calibration results
        undistortion(mtx, dist, args.image_dir, args.image_format)
    else:
        #Find the Camera2Unsprung transformation
        transformationMatrix = external_calib(args.image_dir, args.image_format, args.square_size, args.width, args.height)

        rotation = np.delete(transformationMatrix, 3, 0)    #Trim the transformation matrix
        rotation = np.delete(rotation, 3, 1)                #in order to isolate the rotation matrix

        #Calculate roll, pitch and yaw
        rpy = pr.extrinsic_euler_xyz_from_active_matrix(rotation)


        print("----------------------------------")
        print(rpy)

if __name__ == '__main__':
    main()
    

