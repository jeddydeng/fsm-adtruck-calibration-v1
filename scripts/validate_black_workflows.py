#!/usr/bin/env python3
"""Deterministic SYNTHETIC validation. Never writes any live calibration file."""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'process/joint_extrinsic_station'))
from solve_joint_extrinsics import CAMERA_OPTICAL_TO_LINK, grid_marker_corners, matrix_to_item

RNG=np.random.default_rng(20261105)
FISHEYES=('fisheye_front','fisheye_right','fisheye_rear','fisheye_left')
GMSL=tuple(f'gmsl_video{n}' for n in (0,3,4,5,6,7))
CAMERAS=GMSL+FISHEYES


def dump(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2,allow_nan=False))


def yaml_file(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(yaml.safe_dump(data,sort_keys=False))


def run(folder,name,args,env=None,timeout=900):
    started=time.monotonic()
    with (folder/f'{name}.log').open('w') as log:
        result=subprocess.run([str(x) for x in args],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
    print(f'{name}: exit={result.returncode}, {time.monotonic()-started:.1f}s',flush=True)
    if result.returncode:
        raise RuntimeError(f'{name} failed: see {folder/name}.log\n'+(folder/f'{name}.log').read_text()[-4500:])


def intrinsics(name,size,k,d,fisheye):
    return dict(camera_name=name,image_width=size[0],image_height=size[1],
                distortion_model='equidistant' if fisheye else 'plumb_bob',
                camera_matrix=dict(rows=3,cols=3,data=k.ravel().tolist()),
                distortion_coefficients=dict(rows=1,cols=len(d),data=d.ravel().tolist()))


def rays(size,k,d,fisheye):
    u,v=np.meshgrid(np.arange(size[0],dtype=np.float32),np.arange(size[1],dtype=np.float32))
    p=np.stack([u,v],axis=-1).reshape(-1,1,2)
    if fisheye:
        p=cv2.fisheye.undistortPoints(p.astype(float),k,d)
    else:
        p=cv2.undistortPoints(p.astype(float),k,d)
    return np.column_stack([p.reshape(-1,2),np.ones(len(p))])


def render_plane(texture,size,ray,rotation,translation,physical_size):
    normal=rotation[:,2]
    denom=ray@normal
    depth=(normal@translation)/np.where(abs(denom)>1e-8,denom,1e-8)
    local=(ray*depth[:,None]-translation)@rotation
    mx=(local[:,0]/physical_size[0]*(texture.shape[1]-1)).reshape(size[1],size[0]).astype('f4')
    my=(local[:,1]/physical_size[1]*(texture.shape[0]-1)).reshape(size[1],size[0]).astype('f4')
    mx[depth.reshape(size[1],size[0])<=0]=-100
    return cv2.remap(texture,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=210)


def validate_intrinsics(out):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board=cv2.aruco.CharucoBoard((12,8),.07,.05,dictionary)
    texture=board.generateImage((1200,800),marginSize=0)
    results={}
    for fish,name,size,focal,key in [(False,'gmsl_video0',(1920,1080),1150.,'1'),(True,'fisheye_front',(800,600),290.,'7')]:
        k=np.array([[focal,0,size[0]/2],[0,focal,size[1]/2],[0,0,1.]])
        d=np.zeros(4 if fish else 5)
        ray=rays(size,k,d,fish)
        directory=out/'intrinsic_images'/name;directory.mkdir(parents=True,exist_ok=True)
        for i in range(72 if fish else 48):
            rotation=Rotation.from_euler('xyz',RNG.uniform([-35,-35,-40],[35,35,40]),degrees=True).as_matrix()
            center=np.array([RNG.uniform(-.35,.35),RNG.uniform(-.24,.24),RNG.uniform(.65,1.3) if fish else RNG.uniform(1.1,2.1)])
            if fish and i >= 48:
                # Deliberately observe the lens edges as well as the centre.
                # Production quality checks must not be relaxed for this fixture.
                edge=(i-48)//6
                center=np.array([(-1.05 if edge==0 else 1.05 if edge==1 else 0),
                                 (-.78 if edge==2 else .78 if edge==3 else 0), .65])
            t=center-rotation@np.array([.42,.28,0])
            image=render_plane(texture,size,ray,rotation,t,(.84,.56))
            cv2.imwrite(str(directory/f'synthetic_{i:03}.png'),image)
        env=os.environ.copy();env['CALIBRATION_OUTPUT_ROOT']=str(out/'solver_outputs');env['CAMERA_FISHEYE_SEED_ATTEMPTS']='12'
        run(out,'intrinsics_'+name,[ROOT/'solve_intrinsics.sh',key,directory],env)
        files=sorted((out/'solver_outputs/intrinsics').glob(f'{name}*'))
        if fish:
            info=yaml.safe_load((files[-1]/name/'camera_info.yaml').read_text())
            report=json.loads((files[-1]/name/'report.json').read_text())
            assert report['quality_pass'],report
        else:
            info=yaml.safe_load(next(p for p in reversed(files) if p.suffix=='.yaml').read_text())
        estimated=np.asarray(info['camera_matrix']['data']).reshape(3,3)
        error=float(np.max(abs(np.diag(estimated)[:2]-focal)/focal))
        assert error<.04,(name,error,estimated)
        results[name]=dict(focal_relative_error=error,image_size=size,source='synthetic_rendered_Charuco')
    return results


def draw_grid(size,k,d,fish,camera_from_board,board):
    image=np.full((size[1],size[0]),255,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    rvec=Rotation.from_matrix(camera_from_board[:3,:3]).as_rotvec()
    for tag in range(30):
        points=grid_marker_corners(tag,board)
        if fish:pixel=cv2.fisheye.projectPoints(points.reshape(-1,1,3),rvec,camera_from_board[:3,3],k,d)[0].reshape(4,2)
        else:pixel=cv2.projectPoints(points,rvec,camera_from_board[:3,3],k,d)[0].reshape(4,2)
        lo=np.maximum(np.floor(pixel.min(axis=0)-2).astype(int),0)
        hi=np.minimum(np.ceil(pixel.max(axis=0)+3).astype(int),[size[0],size[1]])
        if np.any(hi-lo<3):continue
        marker=cv2.aruco.generateImageMarker(dictionary,tag,100)
        h=cv2.getPerspectiveTransform(np.float32([[0,0],[99,0],[99,99],[0,99]]),np.float32(pixel-lo))
        warped=cv2.warpPerspective(marker,h,tuple(hi-lo),borderValue=255)
        image[lo[1]:hi[1],lo[0]:hi[0]]=np.minimum(image[lo[1]:hi[1],lo[0]:hi[0]],warped)
    return image


def pcd(path,xyz):
    values=np.column_stack([xyz,np.ones(len(xyz))]).astype('<f4')
    header=f'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n'
    path.write_bytes(header.encode()+values.tobytes())


def validate_joint(out):
    config=yaml.safe_load((ROOT/'config/black_vehicle/station.yaml').read_text())
    initial=yaml.safe_load((ROOT/'config/black_vehicle/initial_extrinsics.yaml').read_text())
    truth={};seed={};camera_info={}
    for item in config['lidars']['sensors']:
        frame=item['frame_id'];v=initial['transforms'][frame]
        m=np.eye(4);m[:3,:3]=Rotation.from_euler('xyz',v['rpy_deg'],degrees=True).as_matrix();m[:3,3]=v['xyz_m']
        truth[frame]=m
        noisy=m.copy()
        if frame!='velodyne_top_base_link':noisy[:3,3]+=RNG.normal(0,.025,3);noisy[:3,:3]=Rotation.from_rotvec([.01,-.01,.02]).as_matrix()@m[:3,:3]
        seed[frame]=matrix_to_item(noisy,'sensor_kit_base_link' if frame=='velodyne_top_base_link' else 'velodyne_top_base_link','SYNTHETIC_seed')
    for j,name in enumerate(CAMERAS):
        optical=np.eye(4);optical[:3,:3]=Rotation.from_euler('xyz',[.008*(j%3),-.008*(j%2),.005*j]).as_matrix();optical[:3,3]=[.018*(j%3),-.02*(j%2),.006*j]
        link=optical@np.linalg.inv(CAMERA_OPTICAL_TO_LINK);truth[name+'/camera_link']=link
        noisy=link.copy();noisy[:3,3]+=[.02,-.01,.02]
        seed[name+'/camera_link']=matrix_to_item(noisy,'sensor_kit_base_link','SYNTHETIC_seed')
        fish=name.startswith('fisheye_');size=(800,600) if fish else (1920,1080);f=320. if fish else 1100.
        k=np.array([[f,0,size[0]/2],[0,f,size[1]/2],[0,0,1.]])
        d=np.zeros(4 if fish else 5);camera_info[name]=(size,k,d,fish,optical)
    data=out/'joint_data';data.mkdir(exist_ok=True)
    for i in range(16):
        directory=data/f'synthetic_{i:03}';(directory/'cameras').mkdir(parents=True,exist_ok=True);(directory/'intrinsics').mkdir(exist_ok=True);(directory/'lidars').mkdir(exist_ok=True)
        board=np.eye(4);board[:3,:3]=Rotation.from_euler('xyz',[180+RNG.uniform(-22,22),RNG.uniform(-22,22),RNG.uniform(-20,20)],degrees=True).as_matrix()
        board[:3,3]=[RNG.uniform(-.28,.28),RNG.uniform(-.25,.25),RNG.uniform(1.1,1.65)]
        center=board[:3,3];normal=board[:3,2]
        sensors={}
        for name,(size,k,d,fish,optical) in camera_info.items():
            image=draw_grid(size,k,d,fish,np.linalg.inv(optical)@board,config['tag_board'])
            filename=f'cameras/synthetic_{name}.png';cv2.imwrite(str(directory/filename),image)
            sensors[name]=dict(saved=True,file=filename,kind='fisheye_camera' if fish else 'gmsl_camera')
            yaml_file(directory/'intrinsics'/f'{name}.yaml',intrinsics(name,size,k,d,fish))
        for item in config['lidars']['sensors']:
            name=item['name'];frame=item['frame_id'];m=truth[frame];local=m[:3,:3].T@(center-m[:3,3]);ln=m[:3,:3].T@normal
            points=local+RNG.normal(0,.1,(400,3));filename=f'lidars/{name}.pcd';pcd(directory/filename,points)
            target=dict(direction='front',center_top_m=center.tolist(),normal_top=normal.tolist(),center_sensor_m=(local+RNG.normal(0,.0001,3)).tolist(),normal_sensor=ln.tolist(),matched_from_roof=False)
            sensors[name]=dict(saved=True,file=filename,kind='lidar',detection=dict(targets=[target]))
        dump(directory/'manifest.json',dict(capture_id=directory.name,sensors=sensors,synthetic=True,software_receive_spread_ms=0))
    initial['transforms']=seed;yaml_file(out/'joint_initial.yaml',initial)
    result=out/'joint_solution.yaml'
    run(out,'joint_solver',[ROOT/'solve_joint.sh','fixed-boards','--initial',out/'joint_initial.yaml','--data-root',data,'--output',result])
    solved=yaml.safe_load(result.read_text());metrics={}
    for frame,expected in truth.items():
        if frame=='velodyne_top_base_link':continue
        assert solved['quality'][frame]['success'],solved['quality'][frame]
        v=solved['transforms'][frame];t=np.array(v['xyz_m']);r=Rotation.from_euler('xyz',v['rpy_deg'],degrees=True).as_matrix()
        terr=float(np.linalg.norm(t-expected[:3,3]));rerr=float(np.degrees(Rotation.from_matrix(r@expected[:3,:3].T).magnitude()))
        assert terr<.04 and rerr<2.,(frame,terr,rerr)
        metrics[frame]=dict(translation_error_m=terr,rotation_error_deg=rerr)
    assert 'traffic_light_front/camera_link' in solved['quality']
    dump(out/'joint_ground_truth.json',{k:v.tolist() for k,v in truth.items()})
    return metrics


def validate_bev(out):
    # Generate four downward-looking equidistant cameras over a textured plane.
    size=(800,600);f=250.;k=np.array([[f,0,400.],[0,f,300.],[0,0,1.]])
    d=np.zeros(4);texture=RNG.integers(45,230,(1800,1800),dtype=np.uint8)
    texture=cv2.GaussianBlur(texture,(5,5),0)
    for _ in range(12000):
        x,y=RNG.integers(0,1800,2);cv2.circle(texture,(int(x),int(y)),int(RNG.integers(2,7)),int(RNG.integers(0,255)),-1)
    yy,xx=np.mgrid[:600,:800];theta=np.hypot(xx-400,yy-300)/f;azimuth=np.arctan2(yy-300,xx-400)
    ray=np.stack([np.sin(theta)*np.cos(azimuth),np.sin(theta)*np.sin(azimuth),np.cos(theta)],axis=-1).reshape(-1,3)
    config=yaml.safe_load((ROOT/'config/black_vehicle/station.yaml').read_text())
    data=out/'bev_data';intr=out/'bev_intrinsics';poses={};extrinsics={}
    for j,name in enumerate(FISHEYES):
        m=np.eye(4);m[:3,:3]=np.diag([1.,-1.,-1.]);m[:3,3]=[.12*(j%2),.12*(j//2),1.8];poses[name]=m
        link=m@np.linalg.inv(CAMERA_OPTICAL_TO_LINK);extrinsics[name+'/camera_link']=matrix_to_item(link,'sensor_kit_base_link','SYNTHETIC_ground_truth')
        yaml_file(intr/name/'camera_info.yaml',intrinsics(name,size,k,d,True))
        dump(intr/name/'report.json',dict(final_fit_rms_px=0,accepted_view_error_px=dict(median=0,p95=0),images=dict(accepted_by_error=48),quality_pass=True))
    selected=[];analytic=[]
    for i in range(14):
        ident=f'synthetic_bev_{i:03}';selected.append(ident);directory=data/ident/'cameras';directory.mkdir(parents=True,exist_ok=True)
        stage=0 if i<3 else 1 if i<9 else 2
        for name,m in poses.items():
            vehicle_ray=ray@m[:3,:3].T
            depth=-m[2,3]/np.where(abs(vehicle_ray[:,2])>1e-7,vehicle_ray[:,2],1e-7)
            ground=vehicle_ray*depth[:,None]+m[:3,3]
            mx=((ground[:,0]+9)*100).reshape(600,800).astype('f4');my=((ground[:,1]+9)*100).reshape(600,800).astype('f4')
            mx[depth.reshape(600,800)<=0]=-100
            image=cv2.remap(texture,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=100)
            cv2.imwrite(str(directory/f'{ident}_{name}.png'),image)
            boards=[]
            for sx,sy in [(1,1),(1,-1),(-1,1),(-1,-1)]:
                b=np.eye(4);b[:3,3]=[sx*(1.0+.45*stage),sy*(1.0+.45*stage),0]
                camera_from_board=np.linalg.inv(m)@b
                boards.append(dict(T_camera_optical_from_board=camera_from_board.tolist(),rms_px=0.,tag_count=30))
            analytic.append(dict(capture_id=ident,camera=name,boards=boards,synthetic_analytic_observations=True))
    selected_file=out/'bev_selected.txt';selected_file.write_text('\n'.join(selected)+'\n')
    yaml_file(out/'bev_extrinsics.yaml',dict(transforms=extrinsics,deploy_allowed=False,synthetic=True))
    # Exercise the actual image detector separately from numerical geometry fixtures.
    run(out,'bev_detect',[ROOT/'solve_bev.sh','detect','--data-root',data,'--group-list',selected_file,'--intrinsics-root',intr,'--output',out/'bev_detection'])
    detection=out/'bev_analytic_observations.jsonl';detection.write_text(''.join(json.dumps(x)+'\n' for x in analytic))
    baseline=out/'bev_baseline'
    run(out,'bev_absolute',[ROOT/'solve_bev.sh','absolute','--data-root',data,'--detections',detection,'--intrinsics-root',intr,'--camera-extrinsics',out/'bev_extrinsics.yaml','--vehicle-roof-xyz-rpy','0,0,0,0,0,0','--output',baseline,'--no-ground-refinement'])
    index=out/'bev_index.txt';index.write_text(''.join(f'{ident}/cameras/{ident}_{name}.png\n' for ident in selected for name in FISHEYES))
    package=out/'bev_relative'
    run(out,'bev_relative',[ROOT/'solve_bev.sh','relative','--data-root',data,'--selected',selected_file,'--raw-index',index,'--baseline',baseline,'--output',package,'--force'])
    run(out,'bev_verify',[ROOT/'solve_bev.sh','verify','--data-root',data,'--package',package,'--baseline',baseline])
    args=[ROOT/'.venv/bin/python',ROOT/'scripts/render_bev.py','--package',package,'--output',out/'bev_render.png']
    for name in FISHEYES:args += ['--'+name.removeprefix('fisheye_'),data/selected[0]/'cameras'/f'{selected[0]}_{name}.png']
    run(out,'bev_render',args)
    report=json.loads((package/'bev_calibration.json').read_text());assert report['all_self_checks_passed'],report['self_checks']
    for name in FISHEYES:
        for axis,bound in [('x',800),('y',600)]:
            arr=np.load(package/f'{name}_bev_map_{axis}.npy');valid=arr>=0
            assert arr.dtype==np.float32 and arr.shape==(512,512) and np.isfinite(arr).all()
            assert (arr[valid]<bound).all()
    return dict(source_size=size,self_checks=report['self_checks'],synthetic_analytic_ground_observations=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--phase',choices=['all','intrinsics','joint','bev'],default='all')
    args=parser.parse_args();out=(args.output or ROOT/'output'/f'simulation_{datetime.now():%Y%m%d_%H%M%S}').resolve();out.mkdir(parents=True,exist_ok=True)
    summary=dict(kind='SYNTHETIC_SOFTWARE_VALIDATION_ONLY',hardware_calibration=False,phase=args.phase,results={})
    try:
        for name,fn in [('intrinsics',validate_intrinsics),('joint',validate_joint),('bev',validate_bev)]:
            if args.phase in ('all',name):summary['results'][name]=fn(out)
        summary['passed']=True
    except BaseException as error:
        summary['passed']=False;summary['error']=str(error);raise
    finally:
        report=out/f'validation_report_{args.phase}.json'
        dump(report,summary)
        print('SYNTHETIC report:',report,flush=True)


if __name__=='__main__':main()
