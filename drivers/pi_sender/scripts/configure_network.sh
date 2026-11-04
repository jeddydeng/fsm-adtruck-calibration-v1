#!/usr/bin/env bash
set -euo pipefail

interface="eth0"
address="192.168.1.186/24"
gateway="192.168.1.1"
dns="192.168.1.1,1.1.1.1"
connection="fsm-pi-001-static"
apply=0

while (($#)); do
    case "$1" in
        --interface) interface="$2"; shift 2 ;;
        --address) address="$2"; shift 2 ;;
        --gateway) gateway="$2"; shift 2 ;;
        --dns) dns="$2"; shift 2 ;;
        --connection) connection="$2"; shift 2 ;;
        --apply) apply=1; shift ;;
        -h|--help)
            echo "Usage: sudo $0 [--interface eth0] [--address 192.168.1.186/24] [--gateway 192.168.1.1] [--dns ADDRS] --apply"
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
if [[ ${EUID} -ne 0 ]]; then
    echo "Run through sudo." >&2
    exit 1
fi
if [[ ${apply} -ne 1 ]]; then
    echo "No network change made. Re-run with --apply from a local console; SSH can disconnect."
    exit 2
fi
if ! nmcli -t -f NAME connection show | grep -Fxq "${connection}"; then
    nmcli connection add type ethernet ifname "${interface}" con-name "${connection}"
fi
nmcli connection modify "${connection}" connection.interface-name "${interface}" \
  ipv4.method manual ipv4.addresses "${address}" ipv4.gateway "${gateway}" \
  ipv4.dns "${dns}" ipv6.method disabled connection.autoconnect yes
nmcli connection up "${connection}"
