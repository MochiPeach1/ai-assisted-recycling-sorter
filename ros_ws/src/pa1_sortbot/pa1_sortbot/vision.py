"""Small local nearest-centroid classifier for the simulator's visual markers."""

import csv
from collections import Counter
from math import sqrt


def rgb_features(red, green, blue):
    total = red + green + blue
    if total < 45:
        return None
    return red / total, green / total, blue / total


class NearestCentroidClassifier:
    def __init__(self, samples, max_distance=0.24):
        grouped = {}
        for red, green, blue, label in samples:
            features = rgb_features(float(red), float(green), float(blue))
            if features is None:
                continue
            grouped.setdefault(str(label), []).append(features)
        if len(grouped) < 2:
            raise ValueError("Training data must contain at least two non-dark classes")
        self.centroids = {
            label: tuple(
                sum(sample[index] for sample in class_samples) / len(class_samples)
                for index in range(3)
            )
            for label, class_samples in grouped.items()
        }
        self.max_distance = float(max_distance)

    @classmethod
    def from_csv(cls, path):
        with open(path, newline="", encoding="utf-8") as stream:
            rows = csv.DictReader(stream)
            expected = {"red", "green", "blue", "label"}
            if not rows.fieldnames or not expected.issubset(rows.fieldnames):
                raise ValueError("Training CSV needs red, green, blue, and label columns")
            samples = [
                (row["red"], row["green"], row["blue"], row["label"])
                for row in rows
            ]
        return cls(samples)

    def classify_rgb(self, red, green, blue):
        features = rgb_features(red, green, blue)
        if features is None:
            return None
        distances = {
            label: sqrt(sum((features[i] - center[i]) ** 2 for i in range(3)))
            for label, center in self.centroids.items()
        }
        label, distance = min(distances.items(), key=lambda item: item[1])
        if distance > self.max_distance:
            return None
        return label

    def classify_pixels(self, pixels, min_votes=5):
        votes = Counter()
        for red, green, blue in pixels:
            label = self.classify_rgb(red, green, blue)
            if label is not None:
                votes[label] += 1
        if not votes:
            return None, 0.0, 0
        label, count = votes.most_common(1)[0]
        total = sum(votes.values())
        confidence = count / total
        if count < min_votes or confidence < 0.6:
            return None, confidence, count
        return label, confidence, count


def sample_center_pixels(image, stride=4):
    """Sample an image center ROI without depending on OpenCV or NumPy."""
    encoding = image.encoding.lower()
    if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
        raise ValueError("Expected rgb8, bgr8, rgba8, or bgra8 image encoding")
    channels = 4 if encoding in ("rgba8", "bgra8") else 3
    if image.width <= 0 or image.height <= 0 or image.step < image.width * channels:
        raise ValueError("Image dimensions or row stride are invalid")
    if len(image.data) < image.step * image.height:
        raise ValueError("Image data is shorter than its declared dimensions")
    x_start, x_end = image.width * 3 // 10, image.width * 7 // 10
    y_start, y_end = image.height * 3 // 10, image.height * 7 // 10
    pixels = memoryview(image.data)
    samples = []
    for y in range(y_start, y_end, stride):
        for x in range(x_start, x_end, stride):
            offset = y * image.step + x * channels
            first, second, third = pixels[offset : offset + 3]
            if encoding.startswith("bgr"):
                red, green, blue = third, second, first
            else:
                red, green, blue = first, second, third
            if max(red, green, blue) - min(red, green, blue) >= 55:
                samples.append((red, green, blue))
    return samples
