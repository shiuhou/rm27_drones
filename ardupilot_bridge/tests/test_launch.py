from ardupilot_bridge.launch import EXPECTED_PARAMS, verify_parameters


def test_flight_requires_verified_parameters():
    params = EXPECTED_PARAMS.copy()
    assert verify_parameters(params)
    params["ARMING_SKIPCHK"] = 1
    assert not verify_parameters(params)
    params = EXPECTED_PARAMS.copy()
    del params["FRAME_TYPE"]
    assert not verify_parameters(params)
    params = EXPECTED_PARAMS.copy()
    params["MOT_THST_EXPO"] = 0.65
    assert not verify_parameters(params)
