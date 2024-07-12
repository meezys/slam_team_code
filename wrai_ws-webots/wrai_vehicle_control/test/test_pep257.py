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

from ament_pep257.main import main, generate_pep257_report, _ament_ignore
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[".", "test"])
    report = generate_pep257_report(
        ". test",
        "",
        ",".join(_ament_ignore),
        "",
        "ament",
        "",
        "",
    )

    messages = []
    for filename, errors in report:
        for error in errors:
            m = f"{filename}:{error['linenumber']}: {error['message']}"
            messages.append(m)

    assert rc == 0, "Found code style errors / warnings" + "\n".join(messages)
