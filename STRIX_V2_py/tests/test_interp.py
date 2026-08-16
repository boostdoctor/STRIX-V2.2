
"""Host-side tests for map interpolation semantics (mirrors firmware bilinear)."""
from __future__ import annotations


def bilinear(v00, v01, v10, v11, cf, rf):
    return (
        (1 - cf) * (1 - rf) * v00
        + cf * (1 - rf) * v01
        + (1 - cf) * rf * v10
        + cf * rf * v11
    )


def test_bilinear_corners():
    assert bilinear(10, 20, 30, 40, 0, 0) == 10
    assert bilinear(10, 20, 30, 40, 1, 0) == 20
    assert bilinear(10, 20, 30, 40, 0, 1) == 30
    assert bilinear(10, 20, 30, 40, 1, 1) == 40


def test_bilinear_center():
    assert abs(bilinear(10, 20, 30, 40, 0.5, 0.5) - 25.0) < 1e-6


def test_edge_clamp_fractions():
    # cf/rf clamped 0..1
    cf = min(1, max(0, -0.2))
    rf = min(1, max(0, 1.5))
    assert cf == 0 and rf == 1


def test_inj_tenths():
    tenths = 25
    assert abs(tenths * 0.1 - 2.5) < 1e-9
