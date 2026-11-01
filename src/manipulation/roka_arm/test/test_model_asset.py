from hashlib import sha256
from pathlib import Path


EXPECTED = "059e5c485f5162659ae76b6492d7a8291bbcbce6ab75d1dbbf555f6e9f7acc95"


def test_model_asset_checksum():
    model = Path(__file__).parents[1] / "models" / "box_marker_best.pt"
    assert model.is_file()
    assert sha256(model.read_bytes()).hexdigest() == EXPECTED
