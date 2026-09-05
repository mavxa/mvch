import math
import unittest

from mvch.navigation import Field, compose, distance, inverse, segment_distance, wrap


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.field = Field(dict(rows=6, cols=6, spacing=1.0))

    def test_field_axes(self):
        self.assertEqual(self.field.poses[6], (-1, 0, 0))
        self.assertEqual(self.field.poses[1], (0, 1, 0))
        self.assertEqual(self.field.nearest(-2.03, 1.04), 13)

    def test_all_pairs_shortest_and_no_row_wrap(self):
        for a in range(36):
            for b in range(36):
                route = self.field.route(a, b)
                self.assertEqual((route[0], route[-1]), (a, b))
                expected = abs(a//6-b//6)+abs(a % 6-b % 6)
                self.assertEqual(len(route)-1, expected)
                for left, right in zip(route, route[1:]):
                    self.assertIn(right, self.field.graph[left])

    def test_obstacle_not_known_by_id(self):
        route = self.field.route(18, 0, points=[(-2.0, 0.0)], clearance=0.5)
        self.assertNotIn(12, route)
        self.assertGreater(len(route), len(self.field.route(18, 0)))
        for a, b in zip(route, route[1:]):
            self.assertGreaterEqual(segment_distance((-2, 0), self.field.poses[a], self.field.poses[b]), 0.5)

    def test_obstacle_between_markers_blocks_edge(self):
        route = self.field.route(6, 0, points=[(-0.5, 0)], clearance=0.4)
        self.assertNotEqual(route, [6, 0])
        self.assertEqual(len(route), 4)

    def test_unreachable_and_unknown(self):
        with self.assertRaises(ValueError):
            self.field.route(0, 35, forbidden_edges=[(0, 1), (0, 6)])
        with self.assertRaises(ValueError):
            self.field.route(-1, 35)
        with self.assertRaises(ValueError):
            self.field.route(0, 36)

    def test_weighted_graph(self):
        field = Field({"nodes": [
            dict(id=0, x=0, y=0, neighbors=[1, 2]),
            dict(id=1, x=0, y=10, neighbors=[3]),
            dict(id=2, x=1, y=0, neighbors=[4]),
            dict(id=4, x=2, y=0, neighbors=[3]),
            dict(id=3, x=3, y=0, neighbors=[])]})
        self.assertEqual(field.route(0, 3), [0, 2, 4, 3])

    def test_localization_from_marker_corrects_drift(self):
        map_base = (-2, 1, 0.7)
        base_marker = (0.015, -0.008, -0.7)
        map_marker = compose(map_base, base_marker)
        odom_base = (3, 1.2, -1.1)
        odom_marker = compose(odom_base, base_marker)
        map_from_odom = compose(map_marker, inverse(odom_marker))
        localized = compose(map_from_odom, odom_base)
        for actual, expected in zip(localized, map_base):
            self.assertAlmostEqual(actual, expected)

    def test_lidar_mount_is_not_front(self):
        # Реальный TF симулятора: laser_merged развёрнут на +120 градусов.
        mount = (0, 0, 2*math.pi/3)
        front_ray = (math.cos(-2*math.pi/3), math.sin(-2*math.pi/3), 0)
        body_point = compose(mount, front_ray)
        self.assertAlmostEqual(body_point[0], 1)
        self.assertAlmostEqual(body_point[1], 0)

    def test_inverse_and_wrap(self):
        for pose in [(1, 2, 0.3), (-4, 0, math.pi), (0, 3, -math.pi/2)]:
            identity = compose(pose, inverse(pose))
            self.assertLess(distance(identity, (0, 0)), 1e-10)
            self.assertAlmostEqual(identity[2], 0)
        self.assertAlmostEqual(wrap(2*math.pi+0.1), 0.1)

    def test_invalid_graph(self):
        for data in [dict(rows=0, cols=6, spacing=1), dict(rows=6, cols=6, spacing=-1),
                     {"nodes": []}, {"nodes": [dict(id=1, x=0, y=0, neighbors=[2])]}]:
            with self.assertRaises(ValueError):
                Field(data)


if __name__ == "__main__":
    unittest.main()

