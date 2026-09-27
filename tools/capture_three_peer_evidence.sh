#!/usr/bin/env bash
# Run once on each physical Ubuntu/Mac host before the three-machine Zenoh test.
# Example: bash tools/capture_three_peer_evidence.sh scout 192.168.8.10 192.168.8.12 ~/pushpak-evidence
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: $0 ROLE OTHER_IP_1 OTHER_IP_2 OUTPUT_DIR" >&2
  exit 2
fi

role=$1
case "$role" in ground|scout|rig) ;; *) echo 'ROLE must be ground, scout or rig' >&2; exit 2;; esac
for address in "$2" "$3"; do
  if [[ ! $address =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
    echo "not an IPv4 address: $address" >&2
    exit 2
  fi
done
mkdir -p "$4"
output="$4/${role}_preflight_$(date -u +%Y%m%dT%H%M%SZ).log"

{
  printf 'ROLE=%s\n' "$role"
  printf 'LOCAL_DATE='; date
  printf 'UTC_DATE='; date -u
  printf 'HOST='; hostname
  printf 'OS='; uname -a
  printf 'COMMIT='; git rev-parse HEAD
  printf 'LOCAL_IPS:\n'
  if command -v ip >/dev/null 2>&1; then ip -brief address; else ifconfig; fi

  printf '\nPGREP_ZENOH_RAW_BEGIN\n'
  if [ "$(uname -s)" = Darwin ]; then
    pgrep_options=-fl
  else
    pgrep_options=-a
  fi
  printf 'COMMAND=pgrep %s zenohd\n' "$pgrep_options"
  if pgrep "$pgrep_options" zenohd; then
    printf 'RESULT=FAIL router running\n'
    exit 1
  else
    code=$?
    printf 'PGREP_EXIT=%s\n' "$code"
    if [ "$code" -ne 1 ]; then
      printf 'RESULT=FAIL pgrep error\n'
      exit 1
    fi
    printf 'RESULT=PASS no zenohd process\n'
  fi
  printf 'PGREP_ZENOH_RAW_END\n'

  for address in "$2" "$3"; do
    printf '\nPING_TARGET=%s\n' "$address"
    ping -c 3 "$address"
  done
  printf '\nPREFLIGHT=PASS\n'
} 2>&1 | tee "$output"

printf 'Evidence saved to %s\n' "$output"
