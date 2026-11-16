#!/usr/bin/env python3
"""
Turns a picture or GIF into a match screen for the brain.

    python tools/make_match_image.py my_photo.png --name team_photo
    python tools/make_match_image.py hype.gif --name hype

It writes src/assets/<name>.c and prints the line to add to src/matchScreens.cpp.
Needs Pillow:  pip install pillow

Options:
    --fill        crop the picture so it fills the whole 480x240 screen
                  (default: keep the whole picture and leave a border)
    --bg RRGGBB   colour used behind see-through pictures (default 000000)
    --width N     make it N pixels wide instead of as big as fits
"""
import argparse
import os
import re
import struct
import sys

from PIL import Image, ImageSequence

SCREEN_W, SCREEN_H = 480, 240


def fit_size(w, h, fill, width):
    if width:
        scale = width / w
    elif fill:
        scale = max(SCREEN_W / w, SCREEN_H / h)
    else:
        scale = min(SCREEN_W / w, SCREEN_H / h)
    return max(1, round(w * scale)), max(1, round(h * scale))


def prepare(frame, size, fill, bg):
    """resize one frame, flatten see-through parts onto bg, crop to the screen"""
    frame = frame.convert("RGBA").resize(size, Image.LANCZOS)
    flat = Image.new("RGBA", frame.size, bg + (255,))
    flat.alpha_composite(frame)
    flat = flat.convert("RGB")
    w, h = flat.size
    if w > SCREEN_W or h > SCREEN_H:
        left = max(0, (w - SCREEN_W) // 2)
        top = max(0, (h - SCREEN_H) // 2)
        flat = flat.crop((left, top, left + min(w, SCREEN_W), top + min(h, SCREEN_H)))
    return flat


def lzw_encode(pixels, min_code_size=8):
    """GIF style LZW compression of a run of palette indexes"""
    clear = 1 << min_code_size
    stop = clear + 1
    max_code = 4095  # start a fresh table just before it's full
    out = bytearray()
    acc = bits = 0
    code_size = min_code_size + 1
    next_code = stop + 1
    table = {}

    def emit(code):
        nonlocal acc, bits
        acc |= code << bits
        bits += code_size
        while bits >= 8:
            out.append(acc & 0xFF)
            acc >>= 8
            bits -= 8

    emit(clear)
    prefix = pixels[0]
    for c in pixels[1:]:
        key = (prefix << 8) | c
        code = table.get(key)
        if code is not None:
            prefix = code
            continue
        emit(prefix)
        if next_code == (1 << code_size) and code_size < 12:
            code_size += 1
        if next_code < max_code:
            table[key] = next_code
            next_code += 1
        else:
            emit(clear)
            table = {}
            code_size = min_code_size + 1
            next_code = stop + 1
        prefix = c
    emit(prefix)
    if next_code == (1 << code_size) and code_size < 12:
        code_size += 1
    emit(stop)
    if bits:
        out.append(acc & 0xFF)
    return bytes(out)


def make_gif(frames, times, for_brain=True):
    """
    Writes an animated GIF by hand instead of with Pillow, because the GIF player in the
    brain's LVGL version throws away the last chunk of every frame if the frame data fills
    the picture exactly (it then glitches and plays slow). So each frame is written one
    pixel short, and the palette is shuffled so that the missing last pixel (which the
    player fills with palette colour 1) comes out the right colour anyway.
    """
    FILL = 1
    w, h = frames[0].size
    out = bytearray()
    for n, (frame, ms) in enumerate(zip(frames, times)):
        pal_img = frame.quantize(colors=256, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
        palette = bytearray((pal_img.getpalette() or [])[:768])
        palette += bytes(768 - len(palette))
        pixels = pal_img.tobytes()

        last = pixels[-1]
        if last != FILL:  # swap palette slots so the last pixel's colour is in slot 1
            swap = bytearray(range(256))
            swap[last], swap[FILL] = FILL, last
            pixels = pixels.translate(bytes(swap))
            a, b = last * 3, FILL * 3
            palette[a:a + 3], palette[b:b + 3] = palette[b:b + 3], palette[a:a + 3]

        if n == 0:
            out += b"GIF89a" + struct.pack("<HHBBB", w, h, 0xF7, FILL, 0) + palette
            out += b"\x21\xFF\x0BNETSCAPE2.0\x03\x01\x00\x00\x00"  # loop forever
        delay = max(2, round((ms if ms >= 20 else 100) / 10))  # hundredths of a second
        out += b"\x21\xF9\x04\x04" + struct.pack("<H", delay) + b"\x00\x00"
        out += b"\x2C" + struct.pack("<HHHH", 0, 0, w, h) + b"\x87" + palette
        data = lzw_encode(pixels[:-1] if for_brain else pixels)
        out += b"\x08"
        for i in range(0, len(data), 255):
            out += bytes([len(data[i:i + 255])]) + data[i:i + 255]
        out += b"\x00"
    out += b"\x3B"
    return bytes(out)


def c_array(data):
    lines = []
    for i in range(0, len(data), 20):
        lines.append("    " + ", ".join("0x%02x" % b for b in data[i:i + 20]) + ",")
    return "\n".join(lines)


def write_c(path, name, comment, cf, w, h, stride, data):
    with open(path, "w") as f:
        f.write(f"""// {comment}
// Made by tools/make_match_image.py - don't edit by hand, run the tool again instead.
#include "liblvgl/lvgl.h"

static const uint8_t {name}_map[] = {{
{c_array(data)}
}};

const lv_image_dsc_t {name} = {{
    .header = {{
        .magic = LV_IMAGE_HEADER_MAGIC,
        .cf = {cf},
        .flags = 0,
        .w = {w},
        .h = {h},
        .stride = {stride},
        .reserved_2 = 0,
    }},
    .data_size = sizeof({name}_map),
    .data = {name}_map,
    .reserved = NULL,
}};
""")


def main():
    ap = argparse.ArgumentParser(description="Make a brain match screen from a picture or GIF")
    ap.add_argument("image")
    ap.add_argument("--name", required=True, help="C name for the picture, e.g. team_photo")
    ap.add_argument("--fill", action="store_true")
    ap.add_argument("--bg", default="000000")
    ap.add_argument("--width", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "src", "assets"))
    args = ap.parse_args()

    if not re.fullmatch(r"[a-z_][a-z0-9_]*", args.name):
        sys.exit("--name must be lowercase letters, numbers and _ only (e.g. team_photo)")
    if not re.fullmatch(r"[0-9a-fA-F]{6}", args.bg):
        sys.exit("--bg must look like 000000 or F7D246")
    bg = tuple(int(args.bg[i:i + 2], 16) for i in (0, 2, 4))

    im = Image.open(args.image)
    size = fit_size(im.width, im.height, args.fill, args.width)
    out_path = os.path.join(args.out, args.name + ".c")
    os.makedirs(args.out, exist_ok=True)
    animated = getattr(im, "n_frames", 1) > 1

    if animated:
        # animations stay as a GIF, the brain plays it with LVGL's GIF player
        frames, times = [], []
        for frame in ImageSequence.Iterator(im):
            times.append(frame.info.get("duration", 100) or 100)
            frames.append(prepare(frame, size, args.fill, bg))
        data = make_gif(frames, times)
        w, h = frames[0].size
        write_c(out_path, args.name, f"{os.path.basename(args.image)}: {w}x{h} GIF, {len(frames)} frames",
                "LV_COLOR_FORMAT_RAW", w, h, 0, data)
    else:
        flat = prepare(im, size, args.fill, bg)
        w, h = flat.size
        data = bytearray()
        raw = flat.tobytes()
        for i in range(0, len(raw), 3):
            r, g, b = raw[i], raw[i + 1], raw[i + 2]
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)  # RGB565, low byte first
            data += bytes((v & 0xFF, v >> 8))
        write_c(out_path, args.name, f"{os.path.basename(args.image)}: {w}x{h} RGB565",
                "LV_COLOR_FORMAT_RGB565", w, h, w * 2, data)

    title = args.name.replace("_", " ").upper()
    print(f"wrote {os.path.normpath(out_path)}  ({w}x{h}, {len(data) // 1024 + 1} KB{', animated' if animated else ''})")
    print("now add these two lines to src/matchScreens.cpp:")
    print(f"    LV_IMAGE_DECLARE({args.name});                      // next to the other LV_IMAGE_DECLARE lines")
    extra = ""
    if bg != (0, 0, 0):  # the colour round the picture has to match too
        extra = f", {'true' if animated else 'false'}, 0x{args.bg.upper()}"
    elif animated:
        extra = ", true"
    print(f'    {{"{title}", &{args.name}{extra}}},      // inside the matchScreens list')


if __name__ == "__main__":
    main()
