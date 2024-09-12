import matplotlib.pyplot as plt
from coordinate_trans import *
import numpy as np
def load_coord():
    pts_calib = [   [  72804.,   68475.,   80817.,   77134.,   64791.,   72804.,   68475.,   80817.,    77134.,   64791.],
                    [ 128206.,  135521.,  130492.,  120891.,  125919.,  128206.,  135521.,  130492.,   120891.,  125919.],
                    [-673114., -673066., -673017., -673075., -673143., -703348., -703229., -703210.,  -703292., -703288.]]
    cam_calib = [   [ 6.87684169e+02,  1.09113437e+03,  2.76473634e+02,  2.69358735e+02,   1.08693727e+03,  7.07266129e+02,  1.10421058e+03,  2.88566469e+02,   2.83578111e+02,  1.09799450e+03],
                    [ 5.10652416e+02,  8.09259087e+02,  8.14636156e+02,  1.82959013e+02,   1.65545492e+02,  5.00510429e+02,  7.98931877e+02,  8.06319697e+02,   1.78368591e+02,  1.53777782e+02],
                    [-7.35942000e+05, -7.35942000e+05, -7.35942000e+05, -7.35942000e+05,  -7.35942000e+05, -7.65942000e+05, -7.65942000e+05, -7.65942000e+05,  -7.65942000e+05, -7.65942000e+05]]
    pts_solved = [  [  72806.5831624,    68507.69620433,   80786.51311201,   77157.04265916,    64763.16486208,   72716.16620995,   68513.98630903,   80819.48036307,    77180.0809051,    64791.28621284],
                    [ 128450.85553573,  135517.65570076,  130373.45326726,  120852.33632539,   125834.69917083,  128469.92123413,  135492.21805919,  130371.87102641,   120920.28523093,  125774.70444932],
                    [-673081.72137465, -673066.00064727, -673033.13478616, -673099.94165016,  -673134.20154161, -703272.46066932, -703256.50171309, -703223.38331183,  -703289.87874281, -703324.7755628 ]]

    pts_calib_stack = PointStack([])
    for idx in range(len(pts_calib[0])):
        pts_calib_stack.append(Point([pts_calib[0][idx], pts_calib[1][idx], pts_calib[2][idx]]))

    cam_calib_stack = PointStack([])
    for idx in range(len(cam_calib[0])):
        cam_calib_stack.append(Point([cam_calib[0][idx], cam_calib[1][idx], cam_calib[2][idx]]))

    pts_solved_stack = PointStack([])
    for idx in range(len(pts_solved[0])):
        pts_solved_stack.append(Point([pts_solved[0][idx], pts_solved[1][idx], pts_solved[2][idx]]))

    return pts_calib_stack, cam_calib_stack, pts_solved_stack

def load_transform():
    Tran_xyz_c2p = np.array([   [-1.50332654e+01,  5.91509551e+00, -8.79848939e-03],
                                [ 6.41354251e+00,  1.50005194e+01, -1.52036815e-03],
                                [-3.96419936e-02,  1.06207599e-01,  1.00629620e+00]])
    org_xyz_c2p = Point([ 73648.98609261, 115261.40962846,  67466.93947463])
    Tran_xyz_p2c, org_xyz_p2c = inv_trans_mat(Tran_xyz_c2p, org_xyz_c2p)
    print("Solved transfer matrix \n{}".format(Tran_xyz_p2c))
    print("Solved transfer origin \n{}".format(org_xyz_p2c.col))
    
    return Tran_xyz_p2c, org_xyz_p2c

def plot_scatter_3d(point_stack_list):

    if hasattr(point_stack_list, '__len__'):
        if all([isinstance(point_stack, PointStack) for point_stack in point_stack_list]):
            n_stack = len(point_stack_list)
        else:
            return
    elif isinstance(point_stack_list, PointStack):
        point_stack_list = [point_stack_list]
        n_stack = 1
    else:
        return

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')


    filled_markers = ('.', 'o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X')

    # For each set of style and range settings, plot n random points in the box
    # defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
    for idx in range(n_stack):
        xs = point_stack_list[idx].cols[0, :]
        ys = point_stack_list[idx].cols[1, :]
        zs = point_stack_list[idx].cols[2, :]
        ax.scatter(xs, ys, zs, marker=filled_markers[idx])

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

def main():
    point_stack_list = load_coord()
    plot_scatter_3d(point_stack_list[0::2])
    print("Error between calib and solved: \n{}".format(point_stack_list[0].cols - point_stack_list[2].cols))
    err_vec_pts_stack = PointStack([])
    for idx in range(point_stack_list[0].np):
        err_vec_pts = Point(point_stack_list[0].index(idx).col - point_stack_list[2].index(idx).col)
        err_vec_pts_stack.append(err_vec_pts)
        print("Abs error in um: {}".format(np.linalg.norm(err_vec_pts.col)))
    Tran_xyz_p2c, org_xyz_p2c = load_transform()
    err_vec_cam_stack = err_vec_pts_stack.trans_3d(Tran_xyz_p2c)
    print("Error on Screen XY in pixels, and SLS Z in um: \n{}".format(err_vec_cam_stack.cols))

if __name__ == '__main__':

    main()