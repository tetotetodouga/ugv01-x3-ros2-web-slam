"""Saved-map metadata and PNG previews, without ROS or imaging dependencies."""
from functools import lru_cache
from pathlib import Path
import math
import re
import struct
from urllib.parse import quote
import zlib

import yaml


def map_name(value):
    if not isinstance(value, str):
        raise ValueError('Введите название карты')
    value = value.strip()
    if not re.fullmatch(r'[\w -]{1,64}', value, re.UNICODE):
        raise ValueError('Название: до 64 букв, цифр, пробелов, дефисов или подчёркиваний')
    return value


def map_files(directory, name):
    directory = Path(directory).resolve()
    path = directory / (map_name(name) + '.yaml')
    if not path.resolve().is_relative_to(directory) or not path.is_file():
        raise ValueError('Карта не найдена')
    try:
        metadata = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as exc:
        raise ValueError('Не удалось прочитать файл карты') from exc
    if not isinstance(metadata, dict) or not isinstance(metadata.get('image'), str):
        raise ValueError('Не удалось прочитать файл карты')
    image = (directory / metadata['image']).resolve()
    if not image.is_relative_to(directory) or not image.is_file():
        raise ValueError('Изображение карты не найдено')
    resolution = float(metadata['resolution'])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Некорректный масштаб карты')
    return path, image, metadata


def read_pgm(data):
    position = 0

    def token():
        nonlocal position
        while position < len(data):
            if data[position] in b' \t\r\n':
                position += 1
            elif data[position] == ord('#'):
                end = data.find(b'\n', position)
                position = len(data) if end < 0 else end + 1
            else:
                break
        start = position
        while position < len(data) and data[position] not in b' \t\r\n#':
            position += 1
        return data[start:position]

    kind = token()
    if kind not in (b'P5', b'P2'):
        raise ValueError('Поддерживаются карты PGM и PNG')
    width, height, maximum = int(token()), int(token()), int(token())
    count = width * height
    if not (0 < width <= 16384 and 0 < height <= 16384 and count <= 16_000_000 and 0 < maximum <= 65535):
        raise ValueError('Некорректный размер карты')
    if kind == b'P5':
        if position >= len(data) or data[position] not in b' \t\r\n':
            raise ValueError('Некорректный заголовок PGM')
        # Consume exactly the header separator: a first pixel may itself be whitespace.
        position += 2 if data[position:position + 2] == b'\r\n' else 1
        size = count * (2 if maximum > 255 else 1)
        pixels = data[position:position + size]
        if len(pixels) != size:
            raise ValueError('Неполное изображение карты')
        if maximum > 255:
            pixels = bytes(round(v[0] * 255 / maximum) for v in struct.iter_unpack('>H', pixels))
        elif maximum != 255:
            pixels = bytes(round(v * 255 / maximum) for v in pixels)
    else:
        pixels = bytes(round(int(token()) * 255 / maximum) for _ in range(count))
    return width, height, pixels


def image_size(data):
    if data.startswith(b'\x89PNG\r\n\x1a\n') and len(data) >= 24:
        return struct.unpack('>II', data[16:24])
    width, height, _ = read_pgm(data)
    return width, height


def map_card(directory, name):
    path, image, metadata = map_files(directory, name)
    width, height = image_size(image.read_bytes())
    modified = max(path.stat().st_mtime_ns, image.stat().st_mtime_ns)
    return {'name': name, 'width': width, 'height': height,
            'resolution': float(metadata['resolution']), 'updated_at': modified / 1e9,
            'available': True, 'preview_url': '/api/maps/preview?name=' + quote(name, safe='') + '&v=' + str(modified)}


def list_map_cards(directory):
    cards = []
    for path in sorted(Path(directory).glob('*.yaml'), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            cards.append(map_card(directory, path.stem))
        except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError):
            cards.append({'name': path.stem, 'available': False, 'preview_url': None})
    return cards


def map_preview(directory, name):
    _, image, metadata = map_files(directory, name)
    return _preview(str(image), image.stat().st_mtime_ns, image.stat().st_size,
                    float(metadata.get('free_thresh', .25)), float(metadata.get('occupied_thresh', .65)),
                    bool(metadata.get('negate', 0)))


@lru_cache(maxsize=48)
def _preview(image_path, _modified, _size, free, occupied, negate):
    data = Path(image_path).read_bytes()
    if data.startswith(b'\x89PNG\r\n\x1a\n'):
        return data
    width, height, pixels = read_pgm(data)
    scale = min(1., 360 / width, 240 / height)
    out_w, out_h = max(1, round(width * scale)), max(1, round(height * scale))
    rows = bytearray()
    for y in range(out_h):
        rows.append(0)
        start_y, end_y = int(y * height / out_h), max(1, int((y + 1) * height / out_h))
        for x in range(out_w):
            start_x, end_x = int(x * width / out_w), max(1, int((x + 1) * width / out_w))
            # Preserve thin walls while reducing a large map.
            values = [pixels[sy * width + sx] for sy in range(start_y, end_y)
                      for sx in range(start_x, end_x)]
            probabilities = [v / 255 if negate else 1 - v / 255 for v in values]
            color = ((218, 229, 236) if max(probabilities) > occupied else
                     (59, 71, 81) if min(probabilities) < free else (27, 34, 41))
            rows.extend(color)

    def chunk(kind, payload):
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind + payload))

    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', out_w, out_h, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(bytes(rows))) + chunk(b'IEND', b''))
