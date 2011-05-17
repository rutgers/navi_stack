#!/bin/bash

if [ $# -lt 2 ]; then
	echo "err: incorrect number of arguments" 1>&2
	echo "usage: label.sh <path> <kernel size>"
	exit 1
fi

path_clr="$1/color"
path_tru="$1/truth"
ker_size="$2"

names=`ls "$path_clr"`
temp=`mktemp`

cat > "$temp" <<HEADER
@RELATION lines

@ATTRIBUTE red   NUMERIC
@ATTRIBUTE green NUMERIC
@ATTRIBUTE blue  NUMERIC
@ATTRIBUTE hue   NUMERIC
@ATTRIBUTE sat   NUMERIC
@ATTRIBUTE val   NUMERIC
@ATTRIBUTE line  {0,1}

@DATA
HEADER

for name in $names; do
	rosrun white_filter label.py "$path_clr/$name" "$path_tru/$name" "$ker_size" >> "$temp"
done

cat "$temp"
