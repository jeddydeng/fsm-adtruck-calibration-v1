"""Hardware-profile regression checks; no devices are opened."""
import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest
import yaml

pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('black_station',ROOT/'process/joint_extrinsic_station/joint_capture_station.py')
station=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=station
spec.loader.exec_module(station)


def test_black_devices_and_ports():
    c=station.load_config(ROOT/'config/black_vehicle/station.yaml')
    assert c['expected_sensor_count']==15
    assert c['usb_capture']['pixel_format']=='UYVY'
    assert [x['device'] for x in c['usb_capture']['cameras']]==[f'/dev/video{n}' for n in (0,3,4,5,6,7)]
    assert c['traffic_camera']['enabled'] is True
    assert c['traffic_camera']['required'] is False
    assert c['fisheye']['gateway_ip']=='192.168.1.187'
    assert [(x['frame_id'],x['msop_port'],x['difop_port']) for x in c['lidars']['sensors']]==[
        ('velodyne_top_base_link',6700,7789),('velodyne_left_base_link',6701,7790),
        ('velodyne_right_base_link',6702,7791),('velodyne_rear_base_link',6699,7788)]
    assert c['lidars']['sensors'][3]['topic'].endswith('/rear/pointcloud_raw_all')


def test_black_tf_matches_deployed_autoware():
    top=station.load_top_transforms(ROOT/'config/black_vehicle/initial_extrinsics.yaml')
    deployed=yaml.safe_load((ROOT/'config/black_vehicle/sensor_kit_calibration.yaml').read_text())['sensor_kit_base_link']
    assert set(top)=={f'velodyne_{n}_base_link' for n in ('top','left','right','rear')}
    for frame,matrix in top.items():
        np.testing.assert_allclose(matrix[:3,3],[deployed[frame][k] for k in ('x','y','z')],atol=1e-9)
        np.testing.assert_allclose(matrix[:3,:3],station.rpy_matrix_deg([np.degrees(deployed[frame][k]) for k in ('roll','pitch','yaw')]),atol=1e-9)


def test_black_profile_does_not_reuse_white_intrinsics():
    c=station.load_config(ROOT/'config/black_vehicle/station.yaml')
    assert Path(c['intrinsics_dir']).resolve()==(ROOT/'config/black_vehicle/intrinsics').resolve()
    assert not list(Path(c['intrinsics_dir']).glob('*.yaml'))


def test_optional_traffic_slot_never_blocks_capture_and_can_return(tmp_path):
    import time
    c=station.load_config(ROOT/'config/black_vehicle/station.yaml')
    c['station']['data_root']=str(tmp_path)
    kinds={x['name']:'lidar' for x in c['lidars']['sensors']}
    kinds.update({x['name']:'gmsl_camera' for x in c['usb_capture']['cameras']})
    kinds.update({x:'fisheye_camera' for x in c['fisheye']['sources'].values()})
    kinds['traffic_light_front']='traffic_camera'
    state=station.SharedState(c,kinds)
    for name,kind in kinds.items():
        if kind=='traffic_camera':continue
        payload=np.zeros((8,8,3),np.uint8)
        if kind=='lidar':
            payload=station.PointCloud2()
            payload.height=1;payload.width=1;payload.point_step=16;payload.row_step=16
            payload.fields=[station.PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))]
            payload.data=np.zeros(4,dtype='<f4').tobytes()
        state.update_sample(station.Sample(name,kind,payload,time.monotonic_ns(),time.time_ns()))
    writer=station.CaptureWriter(c,state,ROOT/'config/black_vehicle/station.yaml',ROOT/'config/black_vehicle/initial_extrinsics.yaml')
    absent=writer.capture()
    assert absent['saved_sensor_count']==14 and absent['complete_required_sensor_set']
    assert absent['missing_optional_sensors']==['traffic_light_front']
    state.update_sample(station.Sample('traffic_light_front','traffic_camera',np.zeros((8,8,3),np.uint8),time.monotonic_ns(),time.time_ns()))
    present=writer.capture()
    assert present['saved_sensor_count']==15 and not present['missing_sensors']
    assert state.status()['sensors']['traffic_light_front']['online']
