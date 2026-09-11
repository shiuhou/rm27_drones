import pytest
from rm_world.probes import CASES, probe


@pytest.mark.parametrize("name", list(CASES))
def test_full_field_contact(name):
    result = probe(name)
    assert result["passed"], result
    assert result["max_force_n"] < 1000
