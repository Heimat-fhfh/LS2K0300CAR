import json
import unittest
from pathlib import Path


CONFIG_KEY = "FORWARD_HEIGHT_COMPENSATION_PX_PER_ROW"


class ForwardHeightCompensationConfigTest(unittest.TestCase):
    def test_all_runtime_configs_define_forward_height_compensation(self):
        repo_root = Path(__file__).resolve().parents[1]
        for config_path in sorted((repo_root / "config").glob("config_*.json")):
            with self.subTest(config=config_path.name):
                config = json.loads(config_path.read_text(encoding="utf-8"))
                self.assertIn(CONFIG_KEY, config)
                self.assertIsInstance(config[CONFIG_KEY], (int, float))
                self.assertEqual(config[CONFIG_KEY], 1.65)


if __name__ == "__main__":
    unittest.main()
