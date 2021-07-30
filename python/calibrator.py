
# %%

import pickle
import numpy as np
import matplotlib.pyplot as plt
import cv2


class PointProjector():

    def __init__(self, camera_mat):

        self.camera_mat = camera_mat
        self.camera_mat_inverse = np.linalg.inv(self.camera_mat[:, :3])
        self.offset = self.camera_mat[:, 3:]
        self.b = self.camera_mat_inverse[2] @ self.offset

    def update(self, camera_mat):
        """ fast function that uses cache if possible
        """
        if np.array_equal(self.camera_mat, camera_mat):
            return
        else:
            self.__init__(camera_mat)

    def pix2obj(self, xy, Z=0): # xy: (n, 2)
        ones = np.expand_dims(np.ones_like(xy[0]), axis=0)
        uv1 = np.concatenate((xy, ones), axis=0)
        a = self.camera_mat_inverse[2] @ uv1
        s = (Z+self.b)/a
        XYZ = self.camera_mat_inverse@(s*uv1 - self.offset)
        return XYZ

    def obj2pix(self, XYZ):
        pix_xys = self.camera_mat[:, :3] @ XYZ + self.offset
        pix_xy = pix_xys[:2] / pix_xys[2:]
        return pix_xy

    @staticmethod
    def test():
        cam_mat = np.array([ # This is a real camera matrix
            [ 1.20030032e+03,  2.47982911e+02,  5.83920422e+02, 8.07622521e+04],
            [-2.30030880e+01, -8.32897510e+02,  1.20491774e+03, 4.11438112e+05],
            [-4.08406072e-02,  3.76333679e-01,  9.25583603e-01, 3.31611357e+02]
        ], dtype=np.float32)
        pp = PointProjector(cam_mat)
        obj = np.random.uniform(0, 100, (3, 1000)).astype(np.float32)
        t = time.time()
        for _ in range(1000):
            pix = pp.obj2pix(obj)
            obj2 = pp.pix2obj(pix, obj[2:3])
        print("time: {}".format(time.time() - t))
        print("mse", np.square(np.subtract(obj, obj2)).mean())
        print("max error", np.max(np.abs(np.subtract(obj, obj2))))

        assert np.isclose(obj, obj2, rtol=1e-3, atol=1e-5).all(), "test failed"



# marker pattern information
import markerboard

# %% load data

with open("captures.pkl", "rb") as f:
    captures = pickle.load(f)

all_positions_bot = np.array([d["pos"] for d in captures], np.float32)
all_ids = [d["marker_ids"] for d in captures]
all_positions_pix = [d["markers_corners"] for d in captures]



# %% calibrate with all 9 images

batch_obj = []
batch_pix = []

for positions_pix, ids in zip(all_positions_pix, all_ids):

    offsets = np.array([[[0,0], [1,0], [1,1], [0,1]]], dtype=np.float32) * markerboard.marker_size
    positions_pix = np.array(positions_pix).reshape((-1, 2))
    positions_obj = (markerboard.positions[ids].astype(np.float32) + offsets).reshape((-1, 2))

    # f, ax = plt.subplots(1, 2)
    # ax[0].plot(positions_pix[:,0], positions_pix[:,1], ".")
    # ax[1].plot(positions_obj[:,0], positions_obj[:,1], ".")
    # plt.show()

    positions_obj = np.concatenate((positions_obj, np.zeros_like(positions_obj[..., :1])), axis=-1)

    batch_obj.append(positions_obj)
    batch_pix.append(positions_pix)

im_shape_cv = (640, 480)

retval, intrinsic, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(batch_obj, batch_pix, im_shape_cv, None, None, )

# Test calibration accuracy
for obj, pix, rvec, tvec in zip(batch_obj, batch_pix, rvecs, tvecs):

    pix_calculated, _ = cv2.projectPoints(obj, rvec, tvec, intrinsic, dist_coeffs)
    pix_calculated = pix_calculated[:,0]

    mse = np.mean(np.square(pix - pix_calculated))
    assert mse < 1
    print("cal mse", mse)


extrinsics = [np.concatenate((np.concatenate((cv2.Rodrigues(rvec)[0], tvec), axis=1),((0,0,0,1),)), axis=0) for rvec, tvec in zip(rvecs, tvecs)]


poses = np.array([-e[:3,:3].T @ np.concatenate((e[:2,3:], [[0]]), 0) for e in extrinsics])[...,:2,  0]


def fit_affine(points_a, points_b):
    """
    Return 3 x 3 affine matrix and the mean squared error resulted by the fit
    similar to cv2.getAffineTransform() but takes more than 3 points as input
    """

    l = len(points_a)

    A = np.zeros((l*2, 6))
    A[:l, 0:2] = points_a
    A[:l, 2] = 1
    A[l:, 3:5] = points_a
    A[l:, 5] = 1

    b = points_b.T.reshape((-1, 1))

    res, _, _, _ = np.linalg.lstsq(A, b)

    m = np.eye(3)
    m[:2] = res.reshape((2,3))

    mse = np.mean(((m[:2,:2] @ points_a.T + m[:2,2:]) - points_b.T)**2)*2

    return m, mse

warp_mat, mse = fit_affine(poses, all_positions_bot)
assert mse < 0.1


pattern = np.array([[0,0,39,39,1], [1,39,39,0,0], [1,1,1,1,1]]).T
pattern_world = (warp_mat @ pattern.T).T
plt.plot(poses[:,0], poses[:,1], "o")
plt.plot(all_positions_bot[:,0], all_positions_bot[:,1], "-o")
for i, obj in enumerate(batch_obj):
    xx, yy, _ = obj.T
    plt.plot(xx, yy, ".")
for i, obj in enumerate(batch_obj):
    obj = obj.copy()
    obj[:, 2] = 1
    xx, yy, _ = warp_mat @ obj.T
    plt.plot(xx, yy, ".")
plt.plot(pattern[:,0], pattern[:,1], "-")
plt.plot(pattern_world[:,0], pattern_world[:,1], "-")
plt.axis("equal")
plt.show()



# %% world to camera

class ModelPixConverter:

    def __init__(self, bot_pos, rvec, tvec, intrinsic, dist_coeffs, warp_mat):
        self.cal_bot_pos = bot_pos
        self.cal_rvec = rvec
        self.cal_tvec = tvec
        self.intrinsic = intrinsic
        self.dist_coeffs = dist_coeffs

        self.warpmat_inv = np.linalg.inv(warp_mat)

    def model_to_pix(self, obj, bot_pos):
        delta = np.expand_dims(self.cal_bot_pos - bot_pos, axis=-1)
        obj2 = (self.warpmat_inv[:,:2] @ (obj[:2] + delta)) + self.warpmat_inv[:,2:]
        obj2[2] = 0
        pix_infred, _ = cv2.projectPoints(obj2, self.cal_rvec, self.cal_tvec, self.intrinsic, self.dist_coeffs)
        return pix_infred[:,0]

    def pix_to_model(self, pix, bot_pos):

        undistorted = cv2.undistortPoints(pix, self.intrinsic, self.dist_coeffs, P=self.intrinsic)[:, 0]

        #TODO
        # delta = np.expand_dims(self.cal_bot_pos - bot_pos, axis=-1)
        # obj2 = (self.warpmat_inv[:,:2] @ (obj[:2] + delta)) + self.warpmat_inv[:,2:]
        # obj2[2] = 0
        # pix_infred, _ = cv2.projectPoints(obj2, self.cal_rvec, self.cal_tvec, self.intrinsic, self.dist_coeffs)
        # return pix_infred[:,0]


# use first capture as calibration
mp = ModelPixConverter(all_positions_bot[0], rvecs[0], tvecs[0],intrinsic, dist_coeffs, warp_mat)

# Test world to camera
p = np.array([[266.68, 314.74, 1]]).T
for obj, pix, rvec, tvec, bot_pos in zip(batch_obj, batch_pix, rvecs, tvecs, all_positions_bot):
    pix_infred, _ = cv2.projectPoints(obj, rvec, tvec, intrinsic, dist_coeffs)
    pix_infred = pix_infred[:,0]
    my_pix_pos = mp.model_to_pix(p, bot_pos)

    plt.plot(pix[:,0], pix[:,1], ".")
    plt.plot(pix_infred[:,0], pix_infred[:,1], ".")
    plt.plot(my_pix_pos[:,0], my_pix_pos[:,1], "x")
    plt.show()

# warp_mat2 = np.eye(4)
# warp_mat2[:2,:2] = warp_mat[:2,:2]
# warp_mat2[:2,-1] = warp_mat[:2,-1]

# for extrinsic in extrinsics:

#     extrinsic2 = np.eye(4)
#     extrinsic2[:3] = extrinsic

#     print(warp_mat @ extrinsic)

# for positions_pix in all_positions_pix:

#     positions_pix = np.array(positions_pix).reshape((-1,2))

#     undistorted = cv2.undistortPoints(positions_pix, intrinsic, dist_coeffs, P=intrinsic)[:, 0]

#     plt.plot(positions_pix[:, 0], positions_pix[:, 1], ".")
#     plt.plot(undistorted[:, 0], undistorted[:, 1], ".")
#     plt.axis("equal")
#     plt.show()

# extrinsics2 = [None] * len(image_files)
# for i, extrinsic in zip(sucess_indices, extrinsics):
#     extrinsics2[i] = extrinsic


