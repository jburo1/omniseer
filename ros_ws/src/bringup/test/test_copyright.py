# Copyright 2015 Open Source Robotics Foundation, Inc.
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
from pathlib import Path

from ament_copyright.main import main


class TestCopyright(unittest.TestCase):
    @unittest.skip("No copyright header has been placed in the generated source file.")
    def test_copyright(self) -> None:
        pkg = Path(__file__).resolve().parents[1]
        rc = main(argv=[str(pkg), str(pkg / "test")])
        self.assertEqual(rc, 0, "Found errors")
