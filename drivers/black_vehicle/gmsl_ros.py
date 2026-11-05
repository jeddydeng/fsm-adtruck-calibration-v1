#!/usr/bin/env python3
"""Publish the six verified PCIe GMSL inputs; no synthetic CameraInfo/TF."""
from pathlib import Path
import signal
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
import yaml


def main():
    root = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((root/'config/black_vehicle/station.yaml').read_text())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('black_vehicle_gmsl')
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    def run(camera, publisher):
        device = camera['device']
        while not stop.is_set():
            cap = None
            try:
                identity = Path('/sys/class/video4linux')/Path(device).name/'device'
                if camera['expected_pci_address'] not in str(identity.resolve()):
                    raise RuntimeError(f'Unexpected PCI device for {device}')
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError(f'Cannot open {device}')
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'UYVY'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
                cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
                while not stop.is_set():
                    ok,image=cap.read()
                    if not ok or image is None:
                        raise RuntimeError(f'No frame from {device}')
                    if image.shape[:2] != (1080,1920):
                        raise RuntimeError(f'Unexpected frame size from {device}: {image.shape}')
                    stamp=node.get_clock().now().to_msg()
                    ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])
                    if not ok:
                        raise RuntimeError('JPEG encode failed')
                    msg=CompressedImage()
                    msg.header.stamp=stamp
                    msg.header.frame_id=camera['name']+'/camera_optical_link'
                    msg.format='bgr8; jpeg compressed bgr8'
                    msg.data=jpeg.tobytes()
                    publisher.publish(msg)
            except Exception as error:
                node.get_logger().warning(str(error))
                stop.wait(1)
            finally:
                if cap is not None:
                    cap.release()
    threads=[]
    for camera in config['usb_capture']['cameras']:
        publisher=node.create_publisher(CompressedImage,
            '/sensing/camera/'+camera['name']+'/image_raw/compressed',qos_profile_sensor_data)
        thread=threading.Thread(target=run,args=(camera,publisher),daemon=True)
        thread.start(); threads.append(thread)
    try:
        while not stop.wait(.2):
            rclpy.spin_once(node,timeout_sec=0)
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=3)
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
