# Pi-capture deployment verification

Date: 2026-08-23 (Asia/Shanghai)

## Deployed paths

- Raspberry Pi gateway: `/home/ubuntu/Pi-5-image/bev/bin/w102_camera_gateway`
- Industrial-computer client: `/home/fsm-lab/Pi-capture/capture_cameras.py`
- Captured images: `/home/fsm-lab/Pi-capture/capture-image/`
- Raspberry Pi rollback backup:
  `/home/ubuntu/Pi-5-image/bev/backups/20260820T202523Z-raw-capture/`

## Protocol behavior

- UDP control command 5 (`CAPTURE_RAW`) requests selected raw streams.
- Streams are 0 FRONT, 1 REAR, 2 LEFT, and 3 RIGHT.
- Replies go only to the requester's temporary UDP port.
- Returned frame ID equals the request nonce, preventing delayed responses from
  being mistaken for the current capture.
- Every UDP fragment has a header CRC and every JPEG has a full-payload CRC.
- Missing directions are retried without enabling the continuous raw stream.
- Four files are written only after all four valid JPEGs have arrived.

## Results

- Raspberry Pi Release build: passed.
- Existing C++ tests: 5/5 passed.
- Python control/CRC/out-of-order/mock-four-camera tests: 3/3 passed on both
  the Mac development copy and the industrial computer.
- Updated gateway service: active, control port 5601 bound, zero restarts.
- Real IPC-to-Pi request/ACK: passed.
- Current physical-camera result: captured mask 0, missing mask 15, because no
  USB fisheye cameras were connected during this deployment. The client exited
  with code 2 and correctly saved zero JPEGs.

Run the real capture after connecting the cameras:

```bash
cd /home/fsm-lab/Pi-capture
python3 capture_cameras.py
```
