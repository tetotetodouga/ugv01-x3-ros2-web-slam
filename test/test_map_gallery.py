"""Saved-map previews must reflect map data and stay inside map storage."""
import struct
import zlib

import pytest

from ugv01_room_explore.map_gallery import list_map_cards, map_card, map_name, map_preview, read_pgm


def write_map(folder, name='First floor', image='room.pgm'):
    (folder / image).write_bytes(b'P5\n# map\n3 2\n255\n' + bytes([0, 254, 205, 10, 32, 254]))
    (folder / (name + '.yaml')).write_text(
        f'image: {image}\nresolution: 0.05\nfree_thresh: 0.196\noccupied_thresh: 0.65\nnegate: 0\n', encoding='utf-8')


def test_binary_pgm_keeps_whitespace_valued_first_pixel():
    assert read_pgm(b'P5\r\n2 1\r\n255\r\n\x0a\x20') == (2, 1, b'\x0a\x20')


def test_ascii_and_sixteen_bit_pgm():
    assert read_pgm(b'P2\n# hello\n2 1\n100\n0 100\n') == (2, 1, b'\x00\xff')
    assert read_pgm(b'P5\n2 1\n65535\n\x00\x00\xff\xff') == (2, 1, b'\x00\xff')


def test_saved_map_metadata_and_real_png_pixels(tmp_path):
    write_map(tmp_path)
    card = map_card(tmp_path, 'First floor')
    assert card['width'] == 3 and card['height'] == 2 and card['resolution'] == .05
    assert 'First%20floor' in card['preview_url']
    png = map_preview(tmp_path, 'First floor')
    assert png.startswith(b'\x89PNG\r\n\x1a\n')
    assert struct.unpack('>II', png[16:24]) == (3, 2)
    offset = png.index(b'IDAT')
    size = struct.unpack('>I', png[offset - 4:offset])[0]
    raw = zlib.decompress(png[offset + 4:offset + 4 + size])
    assert raw[1:4] == bytes([218, 229, 236])  # real occupied pixel
    assert raw[4:7] == bytes([59, 71, 81])  # real free pixel
    assert raw[7:10] == bytes([27, 34, 41])  # real unknown pixel


def test_gallery_keeps_unreadable_cards_identifiable(tmp_path):
    write_map(tmp_path)
    (tmp_path / 'broken.yaml').write_text('image: missing.pgm\nresolution: 0.05\n')
    cards = {c['name']: c for c in list_map_cards(tmp_path)}
    assert cards['First floor']['available']
    assert not cards['broken']['available']


def test_preview_rejects_image_outside_map_storage(tmp_path):
    directory = tmp_path / 'maps'; directory.mkdir()
    (tmp_path / 'private.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (directory / 'escape.yaml').write_text('image: ../private.pgm\nresolution: 0.05\n')
    with pytest.raises(ValueError):
        map_preview(directory, 'escape')


@pytest.mark.parametrize('value', ['../secret', '/tmp/x', 'a\\b', '  ', 'a.b', 'x' * 65])
def test_map_names_reject_paths_and_empty_names(value):
    with pytest.raises(ValueError):
        map_name(value)


def test_map_names_support_readable_labels():
    assert map_name('  Первый этаж  ') == 'Первый этаж'


def test_truncated_map_is_not_a_valid_preview():
    with pytest.raises(ValueError):
        read_pgm(b'P5\n100 100\n255\n\x00')
