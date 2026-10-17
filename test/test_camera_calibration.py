#!/usr/bin/env python
import copy
import unittest

from vio_homing.camera_calibration import validate_calibration


def calibration():
    identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    return dict(
        image_width=640, image_height=480, distortion_model='plumb_bob',
        camera_matrix=dict(rows=3, cols=3,
                           data=[400, 0, 320, 0, 400, 240, 0, 0, 1]),
        rectification_matrix=dict(rows=3, cols=3, data=identity),
        projection_matrix=dict(rows=3, cols=4,
                               data=[400, 0, 320, 0, 0, 400, 240, 0,
                                     0, 0, 1, 0]),
        distortion_coefficients=dict(rows=1, cols=5, data=[0] * 5))


class CalibrationTests(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(validate_calibration(calibration(), 640, 480)
                         ['camera_matrix'][0], 400)

    def test_wrong_resolution(self):
        with self.assertRaises(ValueError):
            validate_calibration(calibration(), 1280, 720)

    def test_bad_matrices(self):
        for key in ('camera_matrix', 'rectification_matrix',
                    'projection_matrix', 'distortion_coefficients'):
            for value in (float('nan'), float('inf'), 'bad'):
                data = calibration()
                data[key]['data'][0] = value
                with self.assertRaises(ValueError):
                    validate_calibration(data, 640, 480)
            data = calibration()
            data[key]['data'].pop()
            with self.assertRaises(ValueError):
                validate_calibration(data, 640, 480)

    def test_bad_model_structure_and_focal_length(self):
        for field, value in (('distortion_model', 'equidistant'),
                             ('camera_matrix', {}),
                             ('image_width', 0)):
            data = copy.deepcopy(calibration())
            data[field] = value
            with self.assertRaises(ValueError):
                validate_calibration(data, 640, 480)
        data = calibration()
        data['camera_matrix']['data'][0] = 0
        with self.assertRaises(ValueError):
            validate_calibration(data, 640, 480)
        with self.assertRaises(ValueError):
            validate_calibration(None, 640, 480)


if __name__ == '__main__':
    unittest.main()
