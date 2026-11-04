# W102 UDP protocol v1

## Data header (`W1UP`, 56 bytes)

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | ASCII `W1UP` |
| 4 | 1 | version (`1`) |
| 5 | 1 | header size (`56`) |
| 6 | 2 | message type |
| 8 | 2 | flags |
| 10 | 2 | stream ID |
| 12 | 8 | boot ID |
| 20 | 8 | frame ID |
| 28 | 8 | monotonic capture timestamp, ns |
| 36 | 4 | full payload size |
| 40 | 4 | full payload CRC-32/IEEE |
| 44 | 2 | fragment index |
| 46 | 2 | fragment count |
| 48 | 2 | this fragment payload size |
| 50 | 2 | reserved |
| 52 | 4 | header CRC-32/IEEE over bytes 0-51 |

Message types: `1=CAMERA_JPEG`, `2=BEV_JPEG`, `3=HEALTH_JSON`,
`4=COVERAGE_PNG`, `5=CONTROL_JSON`.

The 10 Hz `HEALTH_JSON` includes `system_cpu_percent` (whole Pi),
`process_cpu_percent`/`gateway_cpu_percent_total_capacity` (this gateway as a
percentage of all four cores), `memory_used_percent`, `memory_used_mb`,
`memory_total_mb`, `temperature_c`, `bev_active_camera_mask`,
`bev_fresh_camera_mask`, `bev_fusion_fps`, `image_processing_fps` and
`bev_send_fps`. `image_processing_fps` counts only newly completed BEV images;
repeated sends used to maintain the 10 Hz wire cadence are not included. It is
an explicit, descriptive alias of `bev_fusion_fps` for new receivers.

Flags: bit 0 JPEG, bit 1 HELLO replay, bit 2 valid, bit 3 preview-only,
bit 4 visual filling was applied outside the measured BEV coverage, bit 5 the
last completed BEV was repeated to maintain publication cadence, bit 6 one or
more camera directions were unavailable/black, bit 7 adaptive illumination was
applied.

## Control packet (`W1CT`, 24 bytes)

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | ASCII `W1CT` |
| 4 | 1 | version (`1`) |
| 5 | 1 | command |
| 6 | 2 | requested stream mask |
| 8 | 2 | requested reply UDP port |
| 10 | 2 | reserved |
| 12 | 8 | requester nonce |
| 20 | 4 | CRC-32/IEEE over bytes 0-19 |

Commands:

- `1=HELLO`: `stream_mask` selects cached stream IDs for immediate replay.
- `2=SET_FEATURES`: value is in `stream_mask`; bit 0 enables raw camera
  streams, bit 1 enables BEV. Unsupported bits are rejected.
- `3=SET_OUTPUT_FPS`: requested output FPS is in `stream_mask`; the accepted
  range is 10–30 Hz so remote control cannot violate the 10 Hz publication
  requirement.
- `4=SET_JPEG_QUALITY`: requested JPEG quality is in `stream_mask`.
- `5=CAPTURE_RAW`: captures the latest fresh raw frame for each selected camera
  stream and sends the JPEG fragments only to `reply_port`. The four deployed
  stream IDs are `0=FRONT`, `1=REAR`, `2=LEFT`, and `3=RIGHT`. Each returned
  `CAMERA_JPEG` uses the request nonce as its frame ID, so a requester can reject
  delayed data from an older request. The `CONTROL_JSON` reply includes
  `requested_mask`, `captured_mask`, and `missing_mask`. This command does not
  enable the continuous raw-camera feature or change the 10 Hz BEV stream.

For set commands, the gateway sends a `CONTROL_JSON` ACK to `reply_port`, with
the original `nonce`. Parameters apply to the current process; boot defaults are
stored in `${INSTALL_DIR}/config/gateway.env` (`INSTALL_DIR` is recorded in
`/etc/fsm-pi-001/install.conf`). Control datagrams are accepted only
when their IPv4 source equals the configured peer. This source filter is not
cryptographic authentication, so the vehicle network must remain isolated.
