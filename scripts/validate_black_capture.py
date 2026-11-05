#!/usr/bin/env python3
"""Bounded hardware acceptance; start and stop only this checkout's processes."""
import json
import os
from pathlib import Path
import time
from urllib.request import Request, urlopen
from station_manager import ROOT, StationManager


def api(url, capture=False):
    with urlopen(Request(url, data=b'' if capture else None), timeout=15) as response:
        return json.load(response)


def main():
    out=ROOT/'output/hardware_acceptance'
    out.mkdir(parents=True,exist_ok=True)
    os.environ['CALIBRATION_DATA_ROOT']=str(out/'captures')
    results={}
    for traffic_only in (True,False):
        name='traffic_slot' if traffic_only else 'all_sensors'
        manager=StationManager()
        try:
            with manager.lock():
                if manager.entries:
                    raise RuntimeError('Stop this checkout before acceptance testing.')
                manager.start(traffic_only)
            deadline=time.monotonic()+35
            while time.monotonic()<deadline:
                status=api(manager.url+'/api/status')
                required=[v for k,v in status['sensors'].items() if k!='traffic_light_front']
                if all(v['online'] for v in required):break
                time.sleep(.5)
            else:
                raise RuntimeError(f'{name}: required sensors did not become online: {status}')
            traffic=status['sensors']['traffic_light_front']
            assert not traffic['online'] and traffic['optional'] and not traffic['installed']
            result=api(manager.url+'/api/capture',True)
            assert result['saved_sensor_count']==(4 if traffic_only else 14),result
            assert result['complete_required_sensor_set'] and result['missing_optional_sensors']==['traffic_light_front']
            results[name]=dict(status=status,capture=result)
            print(name,': saved',result['saved_sensor_count'],'; missing optional traffic slot does not block',flush=True)
        finally:
            with manager.lock():manager.stop()
    (out/'acceptance.json').write_text(json.dumps(results,indent=2))
    print(out/'acceptance.json')


if __name__=='__main__':main()
