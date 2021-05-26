#!/bin/bash

fixup()
{
    lines=$(sed -rn '/[/][*] *ckwg [+][0-9]+/s/.*ckwg [+]([0-9]+).*/\1/p' "$1")
    (
        cat<<EOF
// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

EOF
    tail -n +$((lines + 1)) "$1"
    ) | cat -s > "$1.tmp"
    mv "$1.tmp" "$1"
}

for f in $(git grep -lE '/[*] *ckwg [+][0-9]+')
    do fixup "$f"
done
