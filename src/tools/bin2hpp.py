#!/usr/bin/env python3
"""Generate a C++ header embedding a binary file as a byte array.

Usage: bin2hpp.py INPUT.bin OUTPUT.hpp NAMESPACE ARRAY_NAME

The build regenerates build/ptp_collection.hpp from the fixture so the uhid
front descriptor can never drift from the device's own bytes.
"""
import sys


def main() -> int:
    if len(sys.argv) != 5:
        print(__doc__)
        return 2
    src, dst, ns, arr = sys.argv[1:5]
    data = open(src, "rb").read()
    body = [
        f"// generated from {src} by tools/bin2hpp.py - do not edit",
        "#pragma once",
        "#include <cstddef>",
        "#include <cstdint>",
        f"namespace {ns} {{",
        f"inline constexpr std::uint8_t {arr}[] {{",
        ",".join(str(b) for b in data) + " };",
        f"inline constexpr std::size_t {arr}_size = sizeof({arr});",
        "}",
    ]
    open(dst, "w").write("\n".join(body) + "\n")
    print(f"wrote {dst}: {len(data)} bytes as {ns}::{arr}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
