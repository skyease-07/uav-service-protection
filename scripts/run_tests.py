"""Run isolated study test suites without invoking archival __main__ writers."""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
suites = {
    'Drones_Revision_Stage7': ['test_confirmation', 'test_protected_lns', 'test_independent_geometry'],
    'Drones_Construction_Objective_Control': ['test_objectives'],
    'Drones_Service_Protection_Split': ['test_split'],
}
for study, modules in suites.items():
    print(f'\n=== {study} ===', flush=True)
    subprocess.run([sys.executable, '-B', '-m', 'unittest', '-v', *modules],
                   cwd=root / study / 'code', check=True)
