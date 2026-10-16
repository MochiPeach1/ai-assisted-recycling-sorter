import unittest
from types import SimpleNamespace

from pa1_sortbot.vision import NearestCentroidClassifier, sample_center_pixels


SAMPLES = [
    (218, 205, 42, "paper"),
    (230, 212, 52, "paper"),
    (38, 96, 226, "plastic"),
    (45, 110, 235, "plastic"),
    (224, 42, 58, "metal"),
    (236, 55, 64, "metal"),
]


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.classifier = NearestCentroidClassifier(SAMPLES)

    def test_classifies_training_colors(self):
        self.assertEqual(self.classifier.classify_rgb(222, 205, 45), "paper")
        self.assertEqual(self.classifier.classify_rgb(41, 104, 230), "plastic")
        self.assertEqual(self.classifier.classify_rgb(228, 48, 61), "metal")

    def test_rejects_dark_and_unknown_colors(self):
        self.assertIsNone(self.classifier.classify_rgb(3, 3, 3))
        self.assertIsNone(self.classifier.classify_rgb(115, 118, 116))

    def test_requires_consistent_pixel_votes(self):
        label, confidence, count = self.classifier.classify_pixels(
            [(40, 100, 230)] * 8 + [(225, 205, 40)] * 2
        )
        self.assertEqual(label, "plastic")
        self.assertAlmostEqual(confidence, 0.8)
        self.assertEqual(count, 8)
        self.assertIsNone(self.classifier.classify_pixels([(40, 100, 230)] * 4)[0])

    def test_samples_rgb_and_bgr_center_crop(self):
        rgb = bytes([222, 205, 45] * 64)
        image = SimpleNamespace(
            encoding="rgb8", width=8, height=8, step=24, data=rgb
        )
        self.assertTrue(sample_center_pixels(image, stride=1))
        bgr = bytes([45, 205, 222] * 64)
        image.encoding = "bgr8"
        image.data = bgr
        pixels = sample_center_pixels(image, stride=1)
        self.assertTrue(all(pixel == (222, 205, 45) for pixel in pixels))

    def test_rejects_short_image_data(self):
        image = SimpleNamespace(
            encoding="rgb8", width=8, height=8, step=24, data=bytes(10)
        )
        with self.assertRaises(ValueError):
            sample_center_pixels(image)


if __name__ == "__main__":
    unittest.main()
