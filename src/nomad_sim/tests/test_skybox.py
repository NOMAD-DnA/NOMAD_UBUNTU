"""Validate the local, licensed six-face background without running MVSim."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image


PACKAGE = Path(__file__).resolve().parents[1]
FACES = ('Front', 'Back', 'Up', 'Down', 'Left', 'Right')


class SkyBoxTests(unittest.TestCase):
    def test_local_faces_are_readable_square_images(self):
        text = (PACKAGE / 'worlds/forest.world.xml').read_text(encoding='utf-8')
        world = ET.fromstring(text.replace('block:class', 'block_class'))
        skies = world.findall("element[@class='skybox']")
        self.assertEqual(len(skies), 1)
        pattern = skies[0].findtext('textures')
        self.assertEqual(pattern, '../assets/skybox/SunSet/SunSet%s.jpg')
        self.assertEqual(pattern.count('%s'), 1)
        sizes = []
        for face in FACES:
            with self.subTest(face=face):
                path = PACKAGE / 'worlds' / (pattern % face)
                self.assertTrue(path.is_file())
                with Image.open(path) as image:
                    image.load()
                    self.assertEqual(image.mode, 'RGB')
                    self.assertEqual(image.width, image.height)
                    self.assertGreaterEqual(image.width, 256)
                    # The original Down face is intentionally almost uniform;
                    # only the five visible sky faces need cloud variation.
                    pixels = np.asarray(image).astype(float)
                    self.assertGreater(pixels.max(), 0)
                    if face != 'Down':
                        self.assertGreater(np.ptp(pixels), 20)
                    sizes.append(image.size)
        self.assertEqual(len(set(sizes)), 1)

    def test_original_attribution_is_packaged(self):
        folder = PACKAGE / 'assets/skybox'
        attribution = (folder / 'ATTRIBUTION.md').read_text(encoding='utf-8')
        original = (folder / 'SunSet/LICENSE.txt').read_text(encoding='utf-8')
        self.assertIn('Heiko Irrgang', attribution)
        self.assertIn('Attribution-ShareAlike 3.0', attribution)
        self.assertIn('creativecommons.org/licenses/by-sa/3.0/', attribution)
        self.assertIn('Attribution-ShareAlike 3.0', original)
        self.assertTrue((folder / 'SunSet/README.txt').is_file())


if __name__ == '__main__':
    unittest.main()
