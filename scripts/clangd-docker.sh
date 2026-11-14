#!/usr/bin/env bash
# clangd for editors: runs inside the x86_64 build image, where the headers are.
[ -f /.dockerenv ] && exec clangd "$@"   # already inside the image (dev container)
exec "$(dirname "$(readlink -f "$0")")/../build.sh" clangd "$@"
