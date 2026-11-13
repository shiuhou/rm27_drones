"""Separate research configuration roots from shared simulation assets."""
from pathlib import Path

from rm27_sim.paths import ROOT as SIM_ROOT

ROOT = Path(__file__).resolve().parents[1]
