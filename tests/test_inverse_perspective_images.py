import importlib.util
import json
import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np


SCRIPT_PATH = Path(__file__).with_name("inverse_perspective_images.py")


def load_module():
    spec = importlib.util.spec_from_file_location("inverse_perspective_images", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class InversePerspectiveImagesTest(unittest.TestCase):
    def test_load_transform_config_reads_flat_matrix_row_major(self):
        module = load_module()

        with TemporaryDirectory() as tmp_dir:
            config_path = Path(tmp_dir) / "vision_transform.json"
            config_path.write_text(
                json.dumps(
                    {
                        "input_size": {"width": 320, "height": 240},
                        "output_size": {"width": 114, "height": 100},
                        "undistort": {"enable": True, "calibration_file": "calibration.yaml"},
                        "inverse_perspective": {
                            "enable": True,
                            "matrix": [1, 2, 3, 4, 5, 6, 7, 8, 9],
                        },
                    }
                ),
                encoding="utf-8",
            )

            config = module.load_transform_config(config_path)

        np.testing.assert_array_equal(
            config.matrix,
            np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=np.float64),
        )
        self.assertEqual(config.input_size, (320, 240))
        self.assertEqual(config.output_size, (114, 100))

    def test_load_calibration_yaml_reads_opencv_matrix_data(self):
        module = load_module()

        with TemporaryDirectory() as tmp_dir:
            yaml_path = Path(tmp_dir) / "calibration.yaml"
            yaml_path.write_text(
                textwrap.dedent(
                    """\
                    %YAML:1.0
                    ---
                    camera_matrix: !!opencv-matrix
                       rows: 3
                       cols: 3
                       dt: d
                       data: [ 150., 0., 160., 0., 151., 115., 0., 0., 1. ]
                    dist_coeffs: !!opencv-matrix
                       rows: 1
                       cols: 5
                       dt: d
                       data: [ 0.2, -0.1, 0.003, -0.002, 0.03 ]
                    image_width: 320
                    image_height: 240
                    """
                ),
                encoding="utf-8",
            )

            calibration = module.load_calibration_yaml(yaml_path)

        np.testing.assert_array_equal(
            calibration.camera_matrix,
            np.array([[150.0, 0.0, 160.0], [0.0, 151.0, 115.0], [0.0, 0.0, 1.0]]),
        )
        np.testing.assert_array_equal(
            calibration.dist_coeffs,
            np.array([[0.2, -0.1, 0.003, -0.002, 0.03]]),
        )
        self.assertEqual(calibration.image_size, (320, 240))

    def test_build_perspective_map_applies_output_to_source_matrix(self):
        module = load_module()
        matrix = np.array([[2.0, 0.0, 10.0], [0.0, 3.0, 20.0], [0.0, 0.0, 1.0]])

        map_x, map_y = module.build_perspective_map(
            matrix=matrix,
            input_size=(320, 240),
            output_size=(3, 2),
        )

        np.testing.assert_array_equal(map_x, np.array([[10.0, 12.0, 14.0], [10.0, 12.0, 14.0]], dtype=np.float32))
        np.testing.assert_array_equal(map_y, np.array([[20.0, 20.0, 20.0], [23.0, 23.0, 23.0]], dtype=np.float32))

    def test_save_coordinate_map_writes_reusable_cpp_arrays(self):
        module = load_module()
        map_x = np.array([[1.0, -1.0], [3.5, 4.5]], dtype=np.float32)
        map_y = np.array([[2.0, -1.0], [6.5, 7.5]], dtype=np.float32)

        with TemporaryDirectory() as tmp_dir:
            output_path = Path(tmp_dir) / "coordinate_map.cpp"

            module.save_coordinate_map(
                output_path=output_path,
                map_x=map_x,
                map_y=map_y,
                config_path=Path("config/vision_transform.json"),
                calibration_path=Path("config/calibration.yaml"),
                input_size=(320, 240),
                output_size=(2, 2),
                apply_undistort=True,
                apply_inverse_perspective=True,
            )

            content = output_path.read_text(encoding="utf-8")

        self.assertIn("namespace vision_transform_coordinate_map", content)
        self.assertIn("constexpr int kInputWidth = 320;", content)
        self.assertIn("constexpr int kInputHeight = 240;", content)
        self.assertIn("constexpr int kOutputWidth = 2;", content)
        self.assertIn("constexpr int kOutputHeight = 2;", content)
        self.assertIn("const float kMapX[kOutputHeight][kOutputWidth]", content)
        self.assertIn("const float kMapY[kOutputHeight][kOutputWidth]", content)
        self.assertIn("const uint8_t kValidMask[kOutputHeight][kOutputWidth]", content)
        self.assertIn("{1.000000f, -1.000000f}", content)
        self.assertIn("{3.500000f, 4.500000f}", content)
        self.assertIn("{2.000000f, -1.000000f}", content)
        self.assertIn("{6.500000f, 7.500000f}", content)
        self.assertIn("{1u, 0u}", content)
        self.assertIn("{1u, 1u}", content)


if __name__ == "__main__":
    unittest.main()
