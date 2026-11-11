"""Verify that the published scientific sources match the research archive."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'provenance/source_sha256.json').read_text())
bad = [p for p, digest in manifest.items()
       if not (root / p).is_file() or hashlib.sha256((root / p).read_bytes()).hexdigest() != digest]
if bad:
    raise SystemExit('Missing or changed sources: ' + ', '.join(bad))
print(f'Verified {len(manifest)} original source/configuration files.')
