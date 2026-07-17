#!/usr/bin/env bash

omni_info() {
  printf 'info: %s\n' "$*" >&2
}

omni_warn() {
  printf 'warning: %s\n' "$*" >&2
}

omni_error() {
  printf 'error: %s\n' "$*" >&2
}

omni_die() {
  omni_error "$*"
  exit 2
}
