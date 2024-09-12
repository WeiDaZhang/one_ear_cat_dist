import matplotlib.pyplot as plt
import numpy as np

from coordinate_trans import *
from coordinate_calib import *

def draw_FOV(sls_point, mat_c2s, width=1376, height=1024):
    FOV_c = PointStack(Point(0, 0), Point(0, height), Point(width, height), Point(width, 0), Point(0, 0))
    #plt.plot(FOV_c.x, FOV_c.y)

    FOV_sls = FOV_c.trans_2d(mat_c2s, sls_point)
    plt.plot(FOV_sls.x, FOV_sls.y)

def main():
    ######################
    # To add z coordinate to a Point object
    pnt_a_2d = Point([1,2])
    print(pnt_a_2d.x)
    print(pnt_a_2d.y)
    print(pnt_a_2d.nd)
    pnt_a_3d = pnt_a_2d.add_z_coord(3)
    print(pnt_a_3d.x)
    print(pnt_a_3d.y)
    print(pnt_a_3d.nd)
    #########################
    # PointStack is always 3d
    # 2d Point when made into stack has been extended with z = 0
    # To change the z of a PointStack
    pnt_a_2d = Point([1,2])
    pnt_b_2d = Point([4,5])
    pntstk = PointStack(pnt_a_2d)
    pntstk.append(pnt_b_2d)
    pntstk = pntstk.add(Point([0,0,3]))

    mat_s2c = np.array([[0.1, 0.9], [-0.5, 0.5]])
    point_u = Point(100, -200, 3)
    point_v = Point(np.array([-2000, 500]))
    point_t = Point(np.array([[2000, -3000, 3]]))

    rot_rad = np.pi / 6
    mat_m2s = np.array([[np.cos(rot_rad), -np.sin(rot_rad)], [np.sin(rot_rad), np.cos(rot_rad)]]) * 0.95
    
    print("Designed transfer matrix {}".format(mat_m2s))

    x_minus = Point(-10000, 0)
    x_plus = Point(10000, 0)
    y_minus = Point(0, -10000)
    y_plus = Point(0, 10000)

    x_axis = PointStack([x_minus])
    x_axis.append(x_plus)
    y_axis = PointStack([y_minus, y_plus])

    x_axis_m = x_axis.trans_2d(mat_m2s, point_t)
    y_axis_m = y_axis.trans_2d(mat_m2s, point_t)

    plt.plot(x_axis.x, x_axis.y)
    plt.plot(y_axis.x, y_axis.y)

    plt.plot(x_axis_m.x, x_axis_m.y)
    plt.plot(y_axis_m.x, y_axis_m.y)

    draw_FOV(point_u, mat_s2c)
    draw_FOV(point_v, mat_s2c)

    point_p_m = Point(-5000, 2000)
    point_p_sls = point_p_m.trans_2d(mat_m2s, point_t)
    plt.plot(point_p_sls.x, point_p_sls.y, 'go')

    point_q_m = Point(-1000, 6000)
    point_q_sls = point_q_m.trans_2d(mat_m2s, point_t)
    plt.plot(point_q_sls.x, point_q_sls.y, 'go')

    point_r_m = Point(2000, 4000)
    point_r_sls = point_r_m.trans_2d(mat_m2s, point_t)
    plt.plot(point_r_sls.x, point_r_sls.y, 'go')

    point_k_m = Point(4000, 2000)
    point_k_sls = point_k_m.trans_2d(mat_m2s, point_t)
    plt.plot(point_k_sls.x, point_k_sls.y, 'go')

    point_l_m = Point(-3000, -4000)
    point_l_sls = point_l_m.trans_2d(mat_m2s, point_t)
    plt.plot(point_l_sls.x, point_l_sls.y, 'go')
    plt.grid()
    plt.show()

    
    point_stack_sls = PointStack(point_p_sls, point_q_sls, point_r_sls, point_k_sls, point_l_sls)
    point_stack_m = PointStack(point_p_m, point_q_m, point_r_m, point_k_m, point_l_m)
    Tran_xy_c2s, org_t_s = point_stack_m.solving_trans_mat_org(point_stack_sls)
    print("Solved transfer matrix {}".format(Tran_xy_c2s))
    print("Solved transfer origin {}".format(org_t_s.col))

    Tran_xy_s2c, org_t_c = inv_trans_mat(Tran_xy_c2s, org_t_s)
    print("Reverse transfer matrix {}".format(Tran_xy_s2c))
    print("Reverse transfer origin {}".format(org_t_c.col))

    point_stack_m_rvsd = point_stack_sls.trans_2d(Tran_xy_s2c, org_t_c)
    for idx in range(point_stack_m_rvsd.np):
        print("Reversed Point in m coordinate {}".format(point_stack_m_rvsd.index(idx).col.T))

    print("Befor perturb {}".format(point_stack_m.cols))
    point_stack_m.perturb(0.1)
    print("After perturb {}".format(point_stack_m.cols))

    bb, cc = point_stack_m.solving_trans_mat_org(point_stack_sls)
    print("Solved transfer matrix {}".format(bb))
    print("Solved transfer origin {}".format(cc.col))

    vec_stack_sls = PointStack(point_p_sls + point_q_sls, point_r_sls - point_k_sls)
    vec_stack_m = PointStack(point_p_m - point_q_m, point_r_m - point_k_m)
    dd = vec_stack_sls.solving_vec_trans_mat(vec_stack_m)
    print(dd)

    #####Test 3D Solving############
    print("3d trans test")
    point_stack_pts0 = PointStack(Point([-18663, 254311, -492489]))
    point_stack_pts0.append(Point([-23122, 261618, -492480]))
    point_stack_pts0.append(Point([-10338, 255982, -492373]))
    point_stack_pts0.append(Point([-14203, 247005, -492420]))
    point_stack_pts0.append(Point([-26988, 252641, -492515]))

    point_stack_pts0.append(Point([-18663, 254311, -522124]))
    point_stack_pts0.append(Point([-23122, 261618, -522115]))
    point_stack_pts0.append(Point([-10338, 255982, -521963]))
    point_stack_pts0.append(Point([-14203, 247005, -522062]))
    point_stack_pts0.append(Point([-26988, 252641, -522111]))

    point_stack_cam0 = PointStack(Point([6.75145030e+02, 5.22631219e+02, -6.89655000e+05]))
    point_stack_cam0.append(Point([1.11776519e+03, 8.05864746e+02, -6.89655000e+05]))
    point_stack_cam0.append(Point([2.52289416e+02, 8.37940249e+02, -6.89655000e+05]))
    point_stack_cam0.append(Point([2.28669167e+02, 2.37916795e+02, -6.89655000e+05]))
    point_stack_cam0.append(Point([1.09176519e+03, 2.04844558e+02, -6.89655000e+05]))

    point_stack_cam0.append(Point([6.93741053e+02, 5.26184394e+02, -7.19655000e+05]))
    point_stack_cam0.append(Point([1.13328942e+03, 8.13863733e+02, -7.19655000e+05]))
    point_stack_cam0.append(Point([2.66765190e+02, 8.47422516e+02, -7.19655000e+05]))
    point_stack_cam0.append(Point([2.43145030e+02, 2.47482254e+02, -7.19655000e+05]))
    point_stack_cam0.append(Point([1.10571735e+03, 2.15297215e+02, -7.19655000e+05]))
    
    Tran_xy_c2p, org_c2p = point_stack_cam0.solving_trans_mat_org(point_stack_pts0, True)
    print("Solved transfer matrix {}".format(Tran_xy_c2p))
    print("Solved transfer origin {}".format(org_c2p.col))

    point_stack_pts_trans0 = point_stack_cam0.trans_3d(Tran_xy_c2p, org_c2p)
    print("CAM and SLS transferred to PTS: \n{}".format(point_stack_pts_trans0.cols))

    Tran_xy_p2c, org_p2c = inv_trans_mat(Tran_xy_c2p, org_c2p)
    point_stack_cam_trans0 = point_stack_pts0.trans_3d(Tran_xy_p2c, org_p2c)
    print("PTS transferred to CAM and SLS: \n{}".format(point_stack_cam_trans0.cols))

    #print("2d Trans")
    #Tran_xy_c2s, org_t_s = point_stack_cam0.solving_trans_mat_org(point_stack_pts0)
    #print("Solved transfer matrix {}".format(Tran_xy_c2s))
    #print("Solved transfer origin {}".format(org_t_s.col))

if __name__ == "__main__":
    main()