"""
Context and event hypervectors, the same values board_server.c trains.

Each HV is 4 uint32 words with HV[31:0] first, which is also the order the
beats go out to the FPGA.
"""

# slots and themes must match load_sequences() in board_server.c
CTX_RED     = 0
CTX_MAGENTA = 1
CTX_GREEN   = 2
CTX_BLUE    = 3
CTX_WHITE   = 4

EVT_D1 = 0
EVT_D2 = 1
EVT_D3 = 2
EVT_D4 = 3
EVT_D5 = 4
EVT_D6 = 5
EVT_D7 = 6

# don't regenerate these; they have to match what the board was trained with
CTX_HV = {
    CTX_RED:     (0xe682c9a3, 0x21a3feff, 0x92ba194a, 0xc6647f01),
    CTX_MAGENTA: (0xf5808b11, 0x81497ed7, 0xe2283d0e, 0xcef57b41),
    CTX_GREEN:   (0x8ff86ad0, 0xebe6d881, 0xb0818a19, 0xe1584820),
    CTX_BLUE:    (0xdda09711, 0xc4593447, 0xe468751e, 0xebb5204f),
    CTX_WHITE:   (0xcfa0cb91, 0xe1e3fcc7, 0xb0a8191a, 0xe3746801),
}

EVT_HV = {
    EVT_D1: (0x7f99b528, 0xbccb3593, 0x0a179ab4, 0x6023c479),
    EVT_D2: (0xd7ec5123, 0x7e4b5073, 0x7a170604, 0x444bcbd4),
    EVT_D3: (0xd5bc55ae, 0x7c4b103b, 0x6e973690, 0x4443cfd8),
    EVT_D4: (0xd5bffca0, 0x1c433002, 0x6f9636b4, 0xe0539910),
    EVT_D5: (0xd5a659a7, 0x7c4b305f, 0x6e9676d1, 0x4452dbf8),
    EVT_D6: (0xd72e59a3, 0x7ccb317b, 0x6e963e11, 0x6463dbf0),
    EVT_D7: (0xe7fbb4ae, 0x3c4b10bb, 0x6a379680, 0x6443c5fc),
}

CTX_NAME = {v: k for k, v in {
    "RED": CTX_RED, "MAGENTA": CTX_MAGENTA, "GREEN": CTX_GREEN,
    "BLUE": CTX_BLUE, "WHITE": CTX_WHITE,
}.items()}

EVT_NAME = {EVT_D1: "D1", EVT_D2: "D2", EVT_D3: "D3", EVT_D4: "D4",
            EVT_D5: "D5", EVT_D6: "D6", EVT_D7: "D7"}

NAME_TO_CTX = {v: k for k, v in CTX_NAME.items()}
NAME_TO_EVT = {v: k for k, v in EVT_NAME.items()}


def digit_to_hv(digit):
    """Digit (1..7, "D1".."D7" or a raw slot 0..6) -> HV tuple."""
    if isinstance(digit, str):
        # accept "D3", "3", "d3"
        s = digit.upper().lstrip("D")
        idx = int(s) - 1
    elif isinstance(digit, int):
        if 1 <= digit <= 7:
            idx = digit - 1        # a digit
        else:
            idx = digit            # otherwise a raw slot
    else:
        raise TypeError(f"digit must be int or str, got {type(digit)}")

    if idx not in EVT_HV:
        raise ValueError(f"digit slot {idx} not provisioned "
                         f"(valid: {sorted(EVT_HV.keys())})")
    return EVT_HV[idx]


def color_to_hv(color):
    """Colour name or CTX_* slot -> HV tuple."""
    if isinstance(color, str):
        key = color.upper()
        if key not in NAME_TO_CTX:
            raise ValueError(f"colour '{color}' not provisioned "
                             f"(valid: {sorted(NAME_TO_CTX.keys())})")
        idx = NAME_TO_CTX[key]
    elif isinstance(color, int):
        idx = color
    else:
        raise TypeError(f"color must be int or str, got {type(color)}")

    if idx not in CTX_HV:
        raise ValueError(f"colour slot {idx} not provisioned "
                         f"(valid: {sorted(CTX_HV.keys())})")
    return CTX_HV[idx]


def hv_to_bytes(hv):
    """HV tuple -> 16 little-endian bytes."""
    out = bytearray(16)
    for i, w in enumerate(hv):
        out[4*i : 4*i+4] = int(w & 0xFFFFFFFF).to_bytes(4, "little")
    return bytes(out)


def hv_pretty(hv):
    return "{" + ", ".join(f"0x{w:08x}" for w in hv) + "}"


if __name__ == "__main__":
    print("== digit_to_hv ==")
    for d in ["D1", "D3", "D7", 5]:
        print(f"  {d!r:>4}  ->  {hv_pretty(digit_to_hv(d))}")

    print("\n== color_to_hv ==")
    for c in ["RED", "blue", "MAGENTA", CTX_WHITE]:
        print(f"  {c!r:>10}  ->  {hv_pretty(color_to_hv(c))}")

    # D3's first word is 0xd5bc55ae, so the first bytes are ae 55 bc d5
    print("\n== byte layout check (D3) ==")
    b = hv_to_bytes(digit_to_hv("D3"))
    print("  first 4 bytes (little-endian HV[31:0]):",
          " ".join(f"{x:02x}" for x in b[:4]),
          "  (expected: ae 55 bc d5)")
