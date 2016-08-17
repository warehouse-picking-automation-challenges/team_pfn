#!/usr/bin/env python

# Copyright 2016 Preferred Networks, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import item_locations
import grab
import shelf_stow_locator
import tote_stow_locator
import get_item
import pick_process
import stow_process


def mkloc():
    return item_locations.ItemLocations({"bin_A": ["jane_eyre_dvd"],
                                         "bin_B": ["jane_eyre_dvd",
                                                   "command_hooks",
                                                   "jane_eyre_dvd"]},
                                        ["i_am_a_bunny_book"])


class TestGrabStrategy(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    def test_select_strategies(self):
        return
        # we cannot check the stuff below if we do not
        # have the correct hardware connected
        s = grab.select_strategies("jane_eyre_dvd", "bin_A")
        self.assertIsInstance(s, list)
        self.assertEqual(len(s), 1)
        self.assertIsInstance(s[0], grab.VacuumGrabbing)


class TestShelfStowLocatorStrategy(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    def test_select_bin2bin_strategy(self):
        s = shelf_stow_locator.select_bin2bin_strategy(self.loc)
        self.assertIsInstance(s, shelf_stow_locator.TempStow)

    def test_select_tote2bin_strategy(self):
        s = shelf_stow_locator.select_tote2bin_strategy(self.loc)
        self.assertIsInstance(s, shelf_stow_locator.GrossPointSpaceStow)


class TestToteStowLocatorStrategy(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    def test_select_strategy(self):
        s = tote_stow_locator.select_strategy(self.loc)
        self.assertIsInstance(s, tote_stow_locator.RandomLocationStow)


class TestGetItemStrategy(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    def test_select_strategy(self):
        s = get_item.select_strategy(self.loc)
        self.assertIsInstance(s, get_item.MoveUntilGrabbablePicking)


class TestPickingStrategy(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    def test_select_strategy(self):
        s = pick_process.select_strategy(self.loc)
        self.assertIsInstance(s, pick_process.SimplePicking)


class TestStowingStrategy(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    def test_select_strategy(self):
        s = stow_process.select_strategy(self.loc)
        self.assertIsInstance(s, stow_process.VisibleGreedyStowing)


class TestItemLocationsShelf(unittest.TestCase):
    def setUp(self):
        self.loc = mkloc()

    # invalid parameters when adding

    def test_add_unknown_item_bin(self):
        with self.assertRaises(ValueError):
            self.loc.put_into_bin("bin_A", "hoge")

    def test_add_unknown_item_tote(self):
        with self.assertRaises(ValueError):
            self.loc.put_into_tote("hoge")

    def test_add_unknown_bin(self):
        with self.assertRaises(ValueError):
            self.loc.put_into_bin("bin_X", "jane_eyre_dvd")

    # invalid parameters when taking

    def test_take_unknown_item_bin(self):
        with self.assertRaises(ValueError):
            self.loc.take_from_bin("bin_A", "hoge")

    def test_take_unknown_item_tote(self):
        with self.assertRaises(ValueError):
            self.loc.take_from_tote("hoge")

    def test_take_unknown_bin(self):
        with self.assertRaises(ValueError):
            self.loc.take_from_bin("bin_X", "jane_eyre_dvd")

    # invalid item moves

    def test_take_unpresent_item_bin(self):
        with self.assertRaises(ValueError):
            self.loc.take_from_bin("bin_A", "scotch_duct_tape")

    def test_add_unpending_item_bin(self):
        with self.assertRaises(ValueError):
            self.loc.put_into_bin("bin_A", "command_hooks")

    def test_take_unpresent_item_tote(self):
        with self.assertRaises(ValueError):
            self.loc.take_from_tote("scotch_duct_tape")

    def test_add_unpending_item_tote(self):
        with self.assertRaises(ValueError):
            self.loc.put_into_tote("command_hooks")

    # valid item moves

    def test_take_item_bin(self):
        self.loc.take_from_bin("bin_A", "jane_eyre_dvd")
        assert self.loc.pending == ["jane_eyre_dvd"]
        assert self.loc.shelf["bin_A"] == []
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [], 
        "bin_B": [
            "jane_eyre_dvd", 
            "command_hooks", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": [
        "i_am_a_bunny_book"
    ]
}""")

    def test_take_item_tote(self):
        self.loc.take_from_tote("i_am_a_bunny_book")
        assert self.loc.pending == ["i_am_a_bunny_book"]
        assert self.loc.tote == []
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [
            "jane_eyre_dvd"
        ], 
        "bin_B": [
            "jane_eyre_dvd", 
            "command_hooks", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": []
}""")

    def test_move_item_bin_bin(self):
        self.loc.take_from_bin("bin_B", "command_hooks")
        self.loc.put_into_bin("bin_A", "command_hooks")
        assert self.loc.pending == []
        assert self.loc.shelf["bin_A"] == ["jane_eyre_dvd", "command_hooks"]
        assert self.loc.shelf["bin_B"] == ["jane_eyre_dvd", "jane_eyre_dvd"]
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [
            "jane_eyre_dvd", 
            "command_hooks"
        ], 
        "bin_B": [
            "jane_eyre_dvd", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": [
        "i_am_a_bunny_book"
    ]
}""")

    def test_move_item_bin_tote(self):
        self.loc.take_from_bin("bin_B", "command_hooks")
        self.loc.put_into_tote("command_hooks")
        assert self.loc.pending == []
        assert self.loc.tote == ["i_am_a_bunny_book", "command_hooks"]
        assert self.loc.shelf["bin_B"] == ["jane_eyre_dvd", "jane_eyre_dvd"]
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [
            "jane_eyre_dvd"
        ], 
        "bin_B": [
            "jane_eyre_dvd", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": [
        "i_am_a_bunny_book", 
        "command_hooks"
    ]
}""")

    def test_move_item_tote_bin(self):
        self.loc.take_from_tote("i_am_a_bunny_book")
        self.loc.put_into_bin("bin_A", "i_am_a_bunny_book")
        assert self.loc.pending == []
        assert self.loc.shelf["bin_A"] == ["jane_eyre_dvd", "i_am_a_bunny_book"]
        assert self.loc.tote == []
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [
            "jane_eyre_dvd", 
            "i_am_a_bunny_book"
        ], 
        "bin_B": [
            "jane_eyre_dvd", 
            "command_hooks", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": []
}""")

    def test_drop_item_bin(self):
        self.loc.take_from_bin("bin_B", "command_hooks")
        self.loc.drop("command_hooks")
        assert self.loc.pending == []
        assert self.loc.shelf["bin_B"] == ["jane_eyre_dvd", "jane_eyre_dvd"]
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [
            "jane_eyre_dvd"
        ], 
        "bin_B": [
            "jane_eyre_dvd", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": [
        "i_am_a_bunny_book"
    ]
}""")

    def test_drop_item_tote(self):
        self.loc.take_from_tote("i_am_a_bunny_book")
        self.loc.drop("i_am_a_bunny_book")
        assert self.loc.pending == []
        assert self.loc.tote == []
        self.assertEqual(self.loc.to_json(), """{
    "bin_contents": {
        "bin_A": [
            "jane_eyre_dvd"
        ], 
        "bin_B": [
            "jane_eyre_dvd", 
            "command_hooks", 
            "jane_eyre_dvd"
        ]
    }, 
    "tote_contents": []
}""")

    def test_used_volume_in_bin(self):
        vol = self.loc.used_volume_in_bin("bin_B")
        self.assertEqual(vol, 359 + 359 + 914)
        vol = self.loc.used_volume_in_bin("bin_A")
        self.assertEqual(vol, 359)

    def test_free_volume_after_add(self):
        ratio = self.loc.free_volume_after_add("command_hooks",
                                               "bin_A")
        already_in_bin = 359.0
        added_volume = 914.0
        bin_volume = float(26*24*42)
        remaining_volume = bin_volume - (2*(already_in_bin + 2*added_volume))
        remaining_ratio = remaining_volume / bin_volume
        self.assertAlmostEqual(ratio, remaining_ratio)

    def test_bin_stow_points(self):
        free_B = self.loc.bin_stow_points("bin_B")
        self.assertEqual(free_B, 15)
        free_A = self.loc.bin_stow_points("bin_A")
        self.assertEqual(free_A, 10)


if __name__ == '__main__':
    unittest.main()
