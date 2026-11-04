"""Dependency-free local HTTP server for the W102 browser dashboard."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
from typing import Any, Callable, Optional
from urllib.parse import parse_qs, urlsplit


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>186 车辆状态与图传</title>
<style>
:root{color-scheme:dark;--bg:#0d1116;--panel:#171d24;--panel2:#1d252e;--muted:#8d99a6;--ok:#58df82;--bad:#ff6b63;--accent:#ffc857;--blue:#63b3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:#eef2f5;font:14px ui-monospace,SFMono-Regular,Menlo,"PingFang SC",monospace}
header{height:58px;padding:0 20px;background:#202832;display:flex;align-items:center;gap:18px}h1{font-size:18px;letter-spacing:.04em;margin:0}.spacer{flex:1}.state,.chip{font-weight:800}.chip{padding:7px 12px;border-radius:18px;background:#11161c}.online,.good{color:var(--ok)}.offline,.badtext{color:var(--bad)}.warn{color:var(--accent)}
main{display:grid;grid-template-columns:minmax(0,1fr) 430px;gap:12px;padding:12px;height:calc(100vh - 58px)}
.view{background:#030405;border-radius:10px;display:flex;align-items:center;justify-content:center;overflow:hidden;position:relative;border:1px solid #222b35}.view img{width:100%;height:100%;object-fit:contain}.waiting{position:absolute;color:var(--accent);text-align:center;font-size:19px}.waiting small{display:block;color:var(--muted);font-size:12px;margin-top:9px}
aside{overflow:auto;padding-right:2px}.card{background:var(--panel);border:1px solid #242e39;border-radius:9px;padding:13px;margin-bottom:10px}.title{color:var(--accent);font-weight:850;margin-bottom:9px;display:flex;justify-content:space-between}.row{display:flex;justify-content:space-between;gap:16px;padding:3px 0}.row span:first-child{color:#b7c0ca}.value{font-weight:760;text-align:right}.muted{color:var(--muted)}.soc{font-size:35px;font-weight:900;color:var(--ok);line-height:1.05}.socline{display:flex;align-items:flex-end;justify-content:space-between;margin:2px 0 10px}.sub{font-size:12px;color:var(--muted)}.divider{height:1px;background:#2b3540;margin:8px 0}.cams{display:grid;grid-template-columns:1fr 1fr;gap:7px}.cam{background:var(--panel2);border:1px solid #303945;border-radius:6px;padding:8px}.cam.ok{border-color:#347849}.cam.bad{border-color:#873e3a}.role{font-weight:800}.age{font-size:11px;color:var(--muted);margin-top:4px}.faults{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px}.fault{display:flex;justify-content:space-between;border-bottom:1px solid #252e38;padding:4px 0}.lampgrid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}.lamp{background:#11161c;border:1px solid #303945;border-radius:6px;padding:7px 4px;text-align:center}.lamp.on{border-color:#cf9e31;color:#ffd36b;background:#2a2416}.small{font-size:11px}
	button,select{border:1px solid #41505f;background:#283440;color:#eef2f5;border-radius:7px;padding:7px 12px;font:inherit}button{cursor:pointer}button.primary{background:#246b42;border-color:#37905a}button.danger{background:#8a2424;border-color:#d14b4b;font-weight:850}button.mute-on{background:#8a2424;border-color:#d14b4b;color:#fff;font-weight:850}button:disabled{opacity:.45;cursor:wait}.modal{display:none;position:fixed;inset:0;background:#05080bcf;z-index:20;align-items:center;justify-content:center;padding:20px}.dialog{width:min(920px,96vw);max-height:94vh;overflow:auto;background:#151c23;border:1px solid #354250;border-radius:12px;padding:18px}.dialog h2{margin:0}.dialoghead{display:flex;align-items:center;gap:12px;margin-bottom:14px}.formgrid{display:grid;grid-template-columns:1fr 1fr;gap:10px 16px}.field label{display:block;color:#b7c0ca;margin-bottom:5px}.field input,.field select{width:100%;background:#0e1318;color:#eef2f5;border:1px solid #3a4653;border-radius:6px;padding:8px}.source-note{grid-column:1/-1;background:#102219;border:1px solid #2d6e47;border-radius:7px;padding:9px;color:#bde9ca}.speaker-test{display:flex;align-items:center;gap:14px;background:#24191a;border:1px solid #6f3636;border-radius:8px;padding:12px;margin-top:14px}.speaker-test .description{flex:1}.events{margin-top:14px}.eventhead,.event{display:grid;grid-template-columns:115px 1fr 150px;gap:8px;align-items:center}.eventhead{color:#b7c0ca;font-size:12px;padding:0 2px;margin-top:9px}.event{margin:7px 0}.event input[type=text],.event input[type=number]{background:#0e1318;color:#eef2f5;border:1px solid #3a4653;border-radius:6px;padding:8px;width:100%}.interval-field{display:flex;align-items:center;gap:6px}.interval-field span{color:#b7c0ca}.actions{display:flex;align-items:center;gap:8px;margin-top:16px}.actions .spacer{flex:1}#voiceMessage{min-height:20px}
@media(max-width:1050px){main{grid-template-columns:1fr;height:auto}.view{height:62vw;min-height:340px}aside{overflow:visible}.formgrid{grid-template-columns:1fr}.eventhead{display:none}.event{grid-template-columns:100px 1fr}.interval-field{grid-column:2}.speaker-test{align-items:stretch;flex-direction:column}}
</style></head><body>
	<header><h1>186 车辆状态与图传</h1><div class="chip good">Pi 192.168.1.186</div><div id="canChip" class="chip offline">CAN 离线</div><div id="socChip" class="chip">SOC N/A</div><div class="spacer"></div><select id="outputMode" title="切换树莓派当前图传模式"><option value="bev">融合 BEV（512×512）</option><option value="raw-all">四路鱼眼原图（1600×1200，10 Hz）</option><option value="front">前摄像头（1600×1200）</option><option value="rear">后摄像头（1600×1200）</option><option value="left">左摄像头（1600×1200）</option><option value="right">右摄像头（1600×1200）</option></select><button id="applyOutputMode" class="primary">切换画面</button><button id="toggleTraffic" title="仅切换本页预览，不改变BEV或鱼眼传输">查看交通灯摄像头</button><button id="captureCameras" class="primary" title="保存前、后、左、右四张1600×1200原始图片">一键保存四路原图</button><button id="voiceMuteQuick">一键静音</button><button id="voiceSettings">语音设置</button><div id="state" class="state offline">无数据</div></header>
<main><section class="view"><img id="bev" alt="BEV"><div id="waiting" class="waiting">等待树莓派数据<small>检查树莓派网线与 UDP 发送链路</small></div></section>
<aside>
<div class="card"><div class="title"><span>车辆状态</span><span id="vehicleFresh" class="sub">N/A</span></div>
<div class="row"><span>控制模式</span><span id="mode" class="value">N/A</span></div><div class="row"><span>点火状态</span><span id="ignition" class="value">N/A</span></div><div class="row"><span>停止状态</span><span id="stopState" class="value">N/A</span></div>
<div class="divider"></div><div class="row"><span>当前档位</span><span id="gear" class="value">N/A</span></div><div class="row"><span>当前车速</span><span id="speed" class="value">N/A</span></div><div class="row"><span>实际加速度</span><span id="accel" class="value">N/A</span></div><div class="row"><span>前轮实际转角</span><span id="angle" class="value">N/A</span></div><div class="row"><span>实际驱动扭矩</span><span id="torque" class="value">N/A</span></div><div class="row"><span>油门状态</span><span id="throttle" class="value warn">协议未提供开度</span></div></div>

<div class="card"><div class="title"><span>动力电池</span><span id="batteryFresh" class="sub">N/A</span></div><div class="socline"><span id="soc" class="soc">N/A</span><span id="charging" class="value">N/A</span></div>
<div class="row"><span>母线电压</span><span id="voltage" class="value">N/A</span></div><div class="row"><span>母线电流</span><span id="current" class="value">N/A</span></div><div class="row"><span>BMS故障等级 / 故障码</span><span id="bmsFault" class="value">N/A</span></div></div>

<div class="card"><div class="title">制动、驻车与转向</div><div class="row"><span>制动状态</span><span id="brakeActive" class="value">N/A</span></div><div class="row"><span>制动油压</span><span id="pressure" class="value">N/A</span></div><div class="row"><span>实际减速度</span><span id="decel" class="value">N/A</span></div><div class="row"><span>EPB驻车状态</span><span id="epb" class="value">N/A</span></div><div class="row"><span>EPS工作状态</span><span id="epsWork" class="value">N/A</span></div></div>

<div class="card"><div class="title">灯光与车身</div><div class="lampgrid"><div id="headlightBox" class="lamp">大灯<br><b id="headlight">N/A</b></div><div id="turnBox" class="lamp">转向<br><b id="turn">N/A</b></div><div id="reverseBox" class="lamp">倒车灯<br><b id="reverse">N/A</b></div><div id="brakeLightBox" class="lamp">刹车灯<br><b id="brakeLight">N/A</b></div><div id="hazardBox" class="lamp">双闪<br><b id="hazard">N/A</b></div><div id="hornBox" class="lamp">喇叭<br><b id="horn">N/A</b></div><div id="voiceBox" class="lamp">对外语音<br><b id="voice">N/A</b></div></div></div>

<div class="card"><div class="title">故障与CAN</div><div class="faults"><div class="fault"><span>VCU</span><b id="vcuFault">N/A</b></div><div class="fault"><span>MCU</span><b id="motorFault">N/A</b></div><div class="fault"><span>EPB</span><b id="epbFault">N/A</b></div><div class="fault"><span>EPS</span><b id="epsFault">N/A</b></div><div class="fault"><span>EHB</span><b id="brakeFault">N/A</b></div><div class="fault"><span>BMS</span><b id="batteryFault">N/A</b></div></div><div class="divider"></div><div class="row"><span>CAN接口 / 波特率</span><span id="canInterface" class="value">N/A</span></div><div class="row"><span>CAN帧累计 / 年龄</span><span id="canFrames" class="value">N/A</span></div><div class="row"><span>反馈报文掩码</span><span id="canMask" class="value">N/A</span></div><div class="row"><span>监听方式</span><span class="value good">仅接收，不发送</span></div></div>

<div class="card"><div class="title"><span>语音与485</span><span id="ioOnline" class="sub">连接中</span></div><div class="row"><span>当前播报优先项</span><span id="voiceActive" class="value">N/A</span></div><div class="row"><span>最近播放</span><span id="voiceLast" class="value">N/A</span></div><div class="row"><span>485发送</span><span id="serialState" class="value">N/A</span></div><div id="ioError" class="small badtext"></div></div>

	<div class="card"><div class="title">树莓派与传输</div><div class="row"><span>当前画面模式</span><span id="streamMode" class="value good">融合 BEV</span></div><div class="row"><span>接收 / 发送 / 处理</span><span id="rates" class="value">N/A</span></div><div class="row"><span>系统 / 程序 CPU</span><span id="cpus" class="value">N/A</span></div><div class="row"><span>内存 / 温度</span><span id="resources" class="value">N/A</span></div><div class="row"><span>数据年龄</span><span id="age">N/A</span></div><div class="row"><span>Pi重启 / 帧缺口</span><span id="transportErrors">0 / 0</span></div><div id="streamMessage" class="small"></div></div>
<div class="card"><div class="title"><span>交通灯识别摄像头（独立预览）</span><span id="trafficOnline" class="sub">连接中</span></div><div class="row"><span>图像 / 标定</span><span id="trafficImage" class="value">N/A</span></div><div class="row"><span>接收频率 / 帧龄</span><span id="trafficRate" class="value">N/A</span></div><div class="row"><span>Pi状态查询</span><span id="trafficQuery" class="value">N/A</span></div><div class="row"><span>ROS 2发布</span><span id="trafficRos" class="value">N/A</span></div><div class="small muted">本卡片及预览按钮不会修改融合BEV或四路鱼眼的传输模式。</div></div>
<div class="card"><div class="title"><span>摄像头</span><span id="captureStatus" class="sub">抓拍就绪</span></div><div id="cams" class="cams"></div></div>
<div class="card muted small"><span id="source">source N/A</span><br><span id="flags">flags N/A</span></div>
</aside></main>
<div id="voiceModal" class="modal"><div class="dialog"><div class="dialoghead"><h2>语音播报设置</h2><div class="spacer"></div><button id="voiceMuteNow">一键静音</button><button id="voiceClose">关闭</button></div>
<div class="formgrid"><div class="field"><label><input id="audioEnabled" type="checkbox"> 启用语音播报</label></div><div class="field"><label for="voiceSource">声音来源</label><select id="voiceSource"></select></div><div id="sourceNote" class="source-note"></div><div class="field"><label for="audioOutput">音频输出源</label><select id="audioOutput"></select></div><div class="field"><label for="voiceRole">中文神经播音角色（20项）</label><select id="voiceRole"></select></div><div class="field"><label for="voiceVolume">播放音量 <span id="volumeLabel">80%</span></label><input id="voiceVolume" type="range" min="0" max="100" step="1"></div><div class="field"><label for="maxVolume">最高音量上限 <span id="maxVolumeLabel">80%</span></label><input id="maxVolume" type="range" min="0" max="100" step="1"></div><div class="field"><label for="playbackRate">快放 / 慢放 <span id="playbackRateLabel">1.00×</span></label><input id="playbackRate" type="range" min="0.5" max="2" step="0.05"></div><div class="field"><label for="voiceRate">神经语音合成语速 <span id="rateLabel">0%</span></label><input id="voiceRate" type="range" min="-50" max="100" step="5"></div></div>
<div class="speaker-test"><div class="description"><b>音响硬件测试</b><div class="sub">播放已验证过的正常中文 MP3，约 10 秒；会遵守上方快慢倍率、播放音量和最高音量上限。</div></div><button id="speakerTest" class="danger">按音量上限播放中文测试音</button></div>
<div class="events"><div class="title">播报内容（优先级由上到下）</div><div class="sub">循环间隔表示同一状态持续时，每隔多少秒再播一次；数值越小，播报越频繁。</div><div class="eventhead"><span>触发状态</span><span>播报文字</span><span>循环间隔（秒）</span></div><div id="eventRows"></div></div>
<div class="actions"><select id="testEvent"><option value="reverse">试听：倒车</option><option value="left">试听：左转</option><option value="right">试听：右转</option><option value="automatic">试听：自动驾驶</option><option value="remote">试听：遥控驾驶</option></select><button id="voiceTest">试听当前设置</button><div class="spacer"></div><span id="voiceMessage" class="sub"></span><button id="voiceSave" class="primary">生成语音并应用</button></div></div></div>
<script>
	const el=id=>document.getElementById(id),fmt=(v,u='',d=1)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d)+u:'N/A';let lastFrame=null,lastStreamMode=null,lastTrafficFrame=null,displaySource='vehicle',voiceConfig=null,voiceOptions=null;const streamNames={bev:'融合 BEV（512×512）','raw-all':'四路鱼眼原图（1600×1200，10 Hz）',front:'前摄像头（1600×1200）',rear:'后摄像头（1600×1200）',left:'左摄像头（1600×1200）',right:'右摄像头（1600×1200）'};
const boolText=v=>v===true?'开启':v===false?'关闭':'N/A';
const modeText=v=>({automatic:'自动驾驶',remote:'遥控器',invalid_or_other:'其他 / 无效'})[v]||'N/A';
const stopText=v=>({none:'无',collision_stop:'撞停',mechanical_emergency_stop:'机械急停',can_emergency_stop:'CAN急停',fault_emergency_stop:'故障急停',unknown:'未知'})[v]||'N/A';
const epbText=v=>({0:'释放过程中',1:'已释放',2:'拉起过程中',3:'已拉起'})[String(v)]||('未知 ('+(v??'N/A')+')');
function mark(id,on,labelOn='开',labelOff='关'){const box=el(id+'Box'),out=el(id);const known=typeof on==='number';out.textContent=known?(on===1?labelOn:labelOff):'N/A';box.className='lamp '+(known&&on===1?'on':'');}
function fault(id,level,code){const out=el(id),known=typeof level==='number'&&typeof code==='number';out.textContent=known?`${level} / ${code}`:'N/A';out.className=known&&(level!==0||code!==0)?'badtext':'good';}
async function update(){try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);const s=await r.json(),vs=s.vehicle_status||{},can=vs.can||{},v=vs.vehicle||{},ctl=v.control||{},mot=v.motion||{},st=v.steering||{},br=v.brake||{},epb=v.parking_brake||{},bat=v.battery||{},lights=v.lights||{};
		el('state').textContent=s.connected?'数据在线':'无数据';el('state').className='state '+(s.connected?'online':'offline');const streamMode=s.output_mode||'bev',imageCount=streamMode==='bev'?s.bev_frames:s.raw_frames;if(displaySource==='vehicle')el('waiting').style.display=imageCount>0?'none':'block';el('streamMode').textContent=streamNames[streamMode]||streamMode;if(streamMode!==lastStreamMode){lastStreamMode=streamMode;lastFrame=null;el('outputMode').value=streamMode;}
el('canChip').textContent=can.online?`CAN在线 ${can.interface||''}`:'CAN离线';el('canChip').className='chip '+(can.online?'online':'offline');el('socChip').textContent='SOC '+fmt(bat.soc_percent,'%');el('socChip').className='chip '+(bat.fresh?'good':'warn');
el('vehicleFresh').textContent=ctl.fresh&&mot.fresh?'实时':'部分数据过期';el('vehicleFresh').className='sub '+(ctl.fresh&&mot.fresh?'good':'warn');el('mode').textContent=modeText(ctl.mode);el('ignition').textContent=boolText(ctl.ignition_on);el('stopState').textContent=stopText(ctl.stop_state);el('gear').textContent=mot.gear||'N/A';el('speed').textContent=fmt(mot.speed_kph,' km/h');el('accel').textContent=fmt(mot.acceleration_mps2,' m/s²',2);el('angle').textContent=fmt(st.front_wheel_angle,'°');el('torque').textContent=fmt(mot.motor_torque,'',0);el('throttle').textContent=mot.throttle_signal_available?(mot.drive_output_active?'有驱动输出':'无驱动输出'):'协议未提供开度';
el('soc').textContent=fmt(bat.soc_percent,'%');el('batteryFresh').textContent=bat.fresh?`实时 · ${fmt(bat.age_ms,' ms',0)}`:'数据过期';el('batteryFresh').className='sub '+(bat.fresh?'good':'warn');el('charging').textContent=bat.charging?'正在充电':'未充电';el('voltage').textContent=fmt(bat.voltage_v,' V');el('current').textContent=fmt(bat.current_a,' A');el('bmsFault').textContent=(typeof bat.error_level==='number'&&typeof bat.fault==='number')?`${bat.error_level} / ${bat.fault}`:'N/A';
el('brakeActive').textContent=br.active?'制动中':'未制动';el('brakeActive').className='value '+(br.active?'warn':'good');el('pressure').textContent=fmt(br.pressure,'',0);el('decel').textContent=fmt(br.deceleration_mps2,' m/s²',2);el('epb').textContent=epbText(epb.state);el('epsWork').textContent=typeof st.work_state==='number'?String(st.work_state):'N/A';
mark('headlight',lights.headlight);mark('reverse',lights.reverse_light);mark('brakeLight',lights.brake_light);mark('hazard',lights.hazard_light);mark('horn',lights.horn);mark('voice',lights.external_voice);const turnMap={off:'关',left:'左转',right:'右转',unknown:'未知'};el('turn').textContent=turnMap[lights.turn_signal]||'N/A';el('turnBox').className='lamp '+(lights.turn_signal&&lights.turn_signal!=='off'?'on':'');
fault('vcuFault',ctl.vcu_fault,ctl.checksum_ok===false?1:0);fault('motorFault',mot.motor_error_level,mot.motor_fault);fault('epbFault',epb.error_level,epb.fault);fault('epsFault',st.error_level,st.fault);fault('brakeFault',br.error_level,br.fault);fault('batteryFault',bat.error_level,bat.fault);
el('canInterface').textContent=can.interface?`${can.interface} / ${fmt((can.bitrate||0)/1000,' kbit/s',0)}`:'N/A';el('canFrames').textContent=typeof can.received_frames==='number'?`${can.received_frames} / ${fmt(can.last_frame_age_ms,' ms',0)}`:'N/A';el('canMask').textContent=typeof can.message_mask==='number'?'0x'+can.message_mask.toString(16).toUpperCase():'N/A';
el('rates').textContent=`${fmt(s.rx_fps,'',1)} / ${fmt(s.send_fps,'',1)} / ${fmt(s.image_processing_fps,' Hz',1)}`;el('cpus').textContent=`${fmt(s.cpu_percent,'%',1)} / ${fmt(s.gateway_cpu_percent,'%',1)}`;el('resources').textContent=`${fmt(s.memory_percent,'%',1)} / ${fmt(s.temperature_c,'°C',1)}`;el('age').textContent=fmt(s.data_age_s,' s',2);el('transportErrors').textContent=`${s.sender_restarts??0} / ${s.frame_gaps??0}`;
el('source').textContent='source '+(s.source||'N/A');el('flags').textContent='flags '+((s.flag_names||[]).join(' ')||'normal');el('cams').innerHTML=(s.cameras||[]).map(c=>`<div class="cam ${c.online?'ok':'bad'}"><div class="role">${c.role} ${c.online?'OK':'OFF'}</div><div class="age">age ${fmt(c.age_ms,' ms',0)}</div></div>`).join('');
		if(displaySource==='vehicle'&&imageCount>0&&s.frame_id!==lastFrame){lastFrame=s.frame_id;el('bev').src='/frame.jpg?source=active&id='+encodeURIComponent(String(s.boot_id)+'-'+String(s.frame_id));}
	}catch(e){el('state').textContent='网页错误';el('state').className='state offline';}}
	async function updateTraffic(){try{const r=await fetch('/api/traffic/status',{cache:'no-store'}),s=await r.json();if(!r.ok)throw Error(s.error||r.status);el('trafficOnline').textContent=s.online?'图像在线':'无新图像';el('trafficOnline').className='sub '+(s.online?'good':'warn');el('trafficImage').textContent=`${s.width||'N/A'}×${s.height||'N/A'} / ${s.calibrated?'已标定':'待标定'}`;el('trafficRate').textContent=`${fmt(s.rx_fps,' Hz',2)} / ${fmt(s.frame_age_s,' s',2)}`;const queryText={streaming:'正常接收',pi_responded_no_image:'Pi已应答，相机无图',pi_unreachable:'Pi未应答',bridge_unavailable:'ROS桥接离线'};el('trafficQuery').textContent=queryText[s.query_state]||s.query_state||'N/A';el('trafficQuery').className='value '+(s.query_state==='streaming'?'good':s.query_state==='pi_unreachable'?'badtext':'warn');el('trafficRos').textContent=s.frames_published!==undefined?`${s.frames_published} 帧 · Sensor QoS`:'N/A';if(displaySource==='traffic'){const ready=s.online&&!!s.frame_id;el('waiting').style.display=ready?'none':'block';if(!ready)el('waiting').innerHTML='等待交通灯摄像头<small>超过5秒会自动询问树莓派状态</small>';if(ready&&s.frame_id!==lastTrafficFrame){lastTrafficFrame=s.frame_id;el('bev').src='/api/traffic/frame.jpg?id='+encodeURIComponent(String(s.frame_id));}}}catch(e){el('trafficOnline').textContent='ROS桥接离线';el('trafficOnline').className='sub badtext';el('trafficQuery').textContent='正在等待服务自动恢复';}}
	function toggleTraffic(){if(displaySource==='vehicle'){displaySource='traffic';lastTrafficFrame=null;el('toggleTraffic').textContent='返回BEV / 鱼眼画面';el('toggleTraffic').className='primary';el('waiting').innerHTML='等待交通灯摄像头<small>仅改变本页显示，不影响BEV或鱼眼传输</small>';el('waiting').style.display='block';updateTraffic();}else{displaySource='vehicle';lastFrame=null;el('toggleTraffic').textContent='查看交通灯摄像头';el('toggleTraffic').className='';el('waiting').innerHTML='等待树莓派数据<small>检查树莓派网线与 UDP 发送链路</small>';el('waiting').style.display='block';update();}}
const eventDefs=[['reverse','倒车'],['left','左转'],['right','右转'],['automatic','自动驾驶'],['remote','遥控驾驶']],eventNames={...Object.fromEntries(eventDefs),preview:'试听'};
function eventLabel(name){return eventNames[name]||'无';}
function updateMuteButtons(muted){for(const id of ['voiceMuteQuick','voiceMuteNow']){const b=el(id);b.textContent=muted?'解除静音':'一键静音';b.className=muted?'mute-on':'';}el('speakerTest').disabled=muted;el('voiceTest').disabled=muted;}
function updateSourceUi(){const original=el('voiceSource').value==='original';el('voiceRole').disabled=original;el('voiceRate').disabled=original;eventDefs.forEach(([key])=>el('text_'+key).disabled=original);el('sourceNote').textContent=original?'正在使用树莓派本地保存的原系统录音。倒车、左转、右转、自动驾驶均为旧录音；旧系统没有遥控驾驶专用录音，该状态使用旧版“车辆行驶”录音。':'正在使用可编辑的中文神经语音；保存时工控机会重新生成语音文件。';el('voiceSave').textContent=original?'应用原系统录音设置':'生成语音并应用';}
function refreshVoiceLabels(){el('volumeLabel').textContent=el('voiceVolume').value+'%';el('maxVolumeLabel').textContent=el('maxVolume').value+'%';el('playbackRateLabel').textContent=Number(el('playbackRate').value).toFixed(2)+'×';el('rateLabel').textContent=(Number(el('voiceRate').value)>0?'+':'')+el('voiceRate').value+'%';}
function fillVoiceForm(bundle){voiceConfig=structuredClone(bundle.config);voiceOptions=bundle;const a=voiceConfig.audio;a.source=a.source||'original';a.muted=!!a.muted;a.max_volume_percent=a.max_volume_percent??80;a.playback_rate=a.playback_rate??1;el('audioEnabled').checked=!!a.enabled;el('voiceSource').innerHTML=(bundle.sources||[{id:'original',label:'原系统录音（树莓派本地，推荐）'},{id:'neural',label:'可编辑中文神经语音'}]).map(x=>`<option value="${x.id}">${x.label}</option>`).join('');el('voiceSource').value=a.source;el('audioOutput').innerHTML=bundle.outputs.map(x=>`<option value="${x.id}">${x.label}</option>`).join('');el('audioOutput').value=a.output;el('voiceRole').innerHTML=bundle.voices.map(x=>`<option value="${x.id}">${x.label}</option>`).join('');el('voiceRole').value=a.voice;el('voiceVolume').value=a.volume_percent;el('maxVolume').value=a.max_volume_percent;el('playbackRate').value=a.playback_rate;el('voiceRate').value=a.rate_percent;el('eventRows').innerHTML=eventDefs.map(([key,label])=>{const x=a.events[key];return `<div class="event"><label><input id="enable_${key}" type="checkbox" ${x.enabled?'checked':''}> ${label}</label><input id="text_${key}" type="text" maxlength="120"><label class="interval-field"><input id="interval_${key}" type="number" min="1" max="300" step="0.5" value="${x.interval_s}" aria-label="${label}循环间隔（秒）"><span>秒</span></label></div>`}).join('');eventDefs.forEach(([key])=>el('text_'+key).value=a.events[key].text);refreshVoiceLabels();updateMuteButtons(a.muted);updateSourceUi();}
function formConfig(){if(!voiceConfig)throw Error('尚未读取树莓派配置');const c=structuredClone(voiceConfig);c.audio.enabled=el('audioEnabled').checked;c.audio.source=el('voiceSource').value;c.audio.output=el('audioOutput').value;c.audio.voice=el('voiceRole').value;c.audio.volume_percent=Number(el('voiceVolume').value);c.audio.max_volume_percent=Number(el('maxVolume').value);c.audio.playback_rate=Number(el('playbackRate').value);c.audio.rate_percent=Number(el('voiceRate').value);eventDefs.forEach(([key])=>{c.audio.events[key].enabled=el('enable_'+key).checked;c.audio.events[key].text=el('text_'+key).value.trim();c.audio.events[key].interval_s=Number(el('interval_'+key).value)});return c;}
async function loadVoice(){try{const r=await fetch('/api/vehicle-io',{cache:'no-store'}),b=await r.json();if(!r.ok||!b.online)throw Error(b.error||r.status);const st=b.status||{},a=st.audio||{},ser=st.serial||{};el('ioOnline').textContent='服务在线';el('ioOnline').className='sub good';el('voiceActive').textContent=a.muted?'已静音':eventLabel(a.selected_event);el('voiceLast').textContent=eventLabel(a.last_event)+(typeof a.last_play_age_ms==='number'?` · ${fmt(a.last_play_age_ms/1000,' s')}`:'');el('serialState').textContent=ser.error?'异常':`${ser.frames_sent??0} 帧 · ${ser.baud??'N/A'} bps`;el('serialState').className='value '+(ser.error?'badtext':'good');el('ioError').textContent=a.error||ser.error||'';updateMuteButtons(!!a.muted);if(!voiceConfig)fillVoiceForm(b);else voiceConfig.audio.muted=!!a.muted;}catch(e){el('ioOnline').textContent='服务离线';el('ioOnline').className='sub badtext';el('ioError').textContent=String(e.message||e);}}
async function postVoice(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}),out=await r.json();if(!r.ok)throw Error(out.error||r.status);return out;}
async function toggleMute(){if(!voiceConfig)throw Error('尚未读取树莓派配置');const muted=!voiceConfig.audio.muted;const out=await postVoice('/api/vehicle-io/mute',{muted});voiceConfig=out.config||voiceConfig;voiceConfig.audio.muted=muted;updateMuteButtons(muted);el('voiceMessage').textContent=muted?'已一键静音，所有自动播报和试听均已停止':'已解除静音';el('voiceMessage').className='sub good';}
	async function captureCameras(){const b=el('captureCameras'),s=el('captureStatus');b.disabled=true;b.textContent='正在抓拍…';s.textContent='正在请求四路最新原图';s.className='sub warn';try{const r=await fetch('/api/cameras/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}),out=await r.json();if(!r.ok)throw Error(out.error||r.status);s.textContent=`已保存 ${out.files.length} 张 · ${out.files[0].split('_').slice(0,2).join('_')}`;s.className='sub good';alert(`四路原图保存完成\n${out.directory}\n\n${out.files.join('\n')}`);}catch(e){s.textContent='抓拍失败';s.className='sub badtext';alert('四路原图抓拍失败：'+String(e.message||e));}finally{b.disabled=false;b.textContent='一键保存四路原图';}}
	async function applyOutputMode(){const b=el('applyOutputMode'),m=el('outputMode').value,s=el('streamMessage');b.disabled=true;s.textContent='正在切换…';s.className='small warn';try{const r=await fetch('/api/stream/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})}),out=await r.json();if(!r.ok)throw Error(out.error||r.status);lastFrame=null;s.textContent='当前运行已切换为 '+(streamNames[out.output_mode]||out.output_mode)+'；重启后自动恢复融合 BEV';s.className='small good';}catch(e){s.textContent='切换失败：'+String(e.message||e);s.className='small badtext';}finally{b.disabled=false;}}
el('voiceSettings').onclick=()=>{el('voiceModal').style.display='flex'};el('voiceClose').onclick=()=>el('voiceModal').style.display='none';el('voiceSource').onchange=updateSourceUi;for(const id of ['voiceVolume','maxVolume','playbackRate','voiceRate'])el(id).oninput=refreshVoiceLabels;el('voiceMuteQuick').onclick=()=>toggleMute().catch(e=>alert(String(e.message||e)));el('voiceMuteNow').onclick=()=>toggleMute().catch(e=>{el('voiceMessage').textContent=String(e.message||e);el('voiceMessage').className='sub badtext'});
	el('captureCameras').onclick=captureCameras;
	el('applyOutputMode').onclick=applyOutputMode;
	el('toggleTraffic').onclick=toggleTraffic;
el('voiceSave').onclick=async()=>{const b=el('voiceSave'),c=formConfig(),original=c.audio.source==='original';b.disabled=true;el('voiceMessage').textContent=original?'正在切换到树莓派本地原系统录音…':'正在离线生成5类中文神经语音…';try{const out=await postVoice('/api/vehicle-io/config',{config:c});voiceConfig=out.config||c;fillVoiceForm({...voiceOptions,config:voiceConfig});el('voiceMessage').textContent=original?'原系统录音及播放设置已应用':'神经语音已缓存并应用到树莓派';el('voiceMessage').className='sub good';}catch(e){el('voiceMessage').textContent=String(e.message||e);el('voiceMessage').className='sub badtext';}finally{b.disabled=false}};
el('voiceTest').onclick=async()=>{const b=el('voiceTest'),c=formConfig(),original=c.audio.source==='original';b.disabled=true;el('voiceMessage').textContent=original?'正在让树莓派直接播放原系统录音…':'正在生成中文神经语音试听…';try{await postVoice('/api/vehicle-io/test',{config:c,event:el('testEvent').value});el('voiceMessage').textContent=original?'原系统录音试听已发送':'神经语音试听已发送';el('voiceMessage').className='sub good';}catch(e){el('voiceMessage').textContent=String(e.message||e);el('voiceMessage').className='sub badtext';}finally{b.disabled=false}};
el('speakerTest').onclick=async()=>{const c=formConfig(),cap=Math.min(c.audio.volume_percent,c.audio.max_volume_percent);if(!confirm(`测试音将按当前设置播放约10秒，实际音量不超过 ${cap}%。请先远离扬声器，是否继续？`))return;const b=el('speakerTest');b.disabled=true;el('voiceMessage').textContent='正在发送固定中文测试音…';try{await postVoice('/api/vehicle-io/speaker-test',{config:c});el('voiceMessage').textContent=`固定中文测试音已发送（实际音量不超过 ${cap}%）`;el('voiceMessage').className='sub good';}catch(e){el('voiceMessage').textContent=String(e.message||e);el('voiceMessage').className='sub badtext';}finally{b.disabled=false}};
update();updateTraffic();loadVoice();setInterval(update,250);setInterval(updateTraffic,500);setInterval(loadVoice,1000);
</script></body></html>"""


def json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return str(value)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class WebDashboardServer(threading.Thread):
    def __init__(self, bind_host: str, port: int,
                 status_provider: Callable[[], dict[str, Any]],
                 frame_provider: Callable[[str], tuple[Optional[bytes], int]],
                 io_client: Optional[Any] = None,
                 capture_client: Optional[Any] = None,
                 stream_status_provider: Optional[Callable[[], dict[str, Any]]] = None,
                 stream_control: Optional[Any] = None,
                 traffic_camera: Optional[Any] = None) -> None:
        super().__init__(name="w102-web-dashboard", daemon=True)
        self.bind_host = bind_host
        self.port = port
        self.status_provider = status_provider
        self.frame_provider = frame_provider
        self.io_client = io_client
        self.capture_client = capture_client
        self.stream_status_provider = stream_status_provider
        self.stream_control = stream_control
        self.traffic_camera = traffic_camera
        self.ready_event = threading.Event()
        self.error: Optional[BaseException] = None
        self.server: Optional[ReusableThreadingHTTPServer] = None

    @property
    def bound_port(self) -> int:
        return self.server.server_port if self.server is not None else self.port

    def run(self) -> None:
        owner = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "W102Dashboard/1"

            def do_GET(self) -> None:  # noqa: N802 (stdlib API)
                parsed = urlsplit(self.path)
                path = parsed.path
                query = parse_qs(parsed.query)
                if path == "/":
                    self._send(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/api/status":
                    body = json.dumps(json_safe(owner.status_provider()),
                                      ensure_ascii=False,
                                      separators=(",", ":")).encode("utf-8")
                    self._send(200, body, "application/json; charset=utf-8")
                elif path == "/api/stream/status":
                    if owner.stream_status_provider is None:
                        self._json(503, {"error": "图传状态接口未初始化"})
                    else:
                        self._json(200, owner.stream_status_provider())
                elif path == "/api/traffic/status":
                    if owner.traffic_camera is None:
                        self._json(503, {"online": False,
                                         "error": "交通灯相机接口未初始化"})
                    else:
                        self._json(200, owner.traffic_camera.status())
                elif path == "/api/traffic/frame.jpg":
                    if owner.traffic_camera is None:
                        self._send(503, b"traffic camera unavailable\n", "text/plain")
                    else:
                        frame, _frame_id = owner.traffic_camera.frame()
                        if frame is None:
                            self._send(503, b"frame not ready\n", "text/plain")
                        else:
                            self._send(200, frame, "image/jpeg")
                elif path == "/frame.jpg" or (
                        path.startswith("/api/stream/frame/") and
                        path.endswith(".jpg")):
                    if path == "/frame.jpg":
                        source = query.get("source", ["active"])[0]
                    else:
                        source = path[len("/api/stream/frame/"):-len(".jpg")]
                    if source not in ("active", "bev", "raw-all", "front",
                                      "rear", "left", "right"):
                        self._json(400, {"error": "未知图像源"})
                        return
                    frame, _frame_id = owner.frame_provider(source)
                    if frame is None:
                        self._send(503, b"frame not ready\n", "text/plain; charset=utf-8")
                    else:
                        self._send(200, frame, "image/jpeg")
                elif path == "/api/stream/mjpeg":
                    source = query.get("source", ["active"])[0]
                    if source not in ("active", "bev", "raw-all", "front",
                                      "rear", "left", "right"):
                        self._json(400, {"error": "未知图像源"})
                        return
                    try:
                        fps = max(1.0, min(30.0, float(query.get("fps", ["10"])[0])))
                    except ValueError:
                        self._json(400, {"error": "fps 参数无效"})
                        return
                    self._mjpeg(source, fps)
                elif path == "/api/vehicle-io":
                    if owner.io_client is None:
                        self._json(503, {"online": False, "error": "语音控制未初始化"})
                    else:
                        self._json(200, owner.io_client.bundle())
                else:
                    self._send(404, b"Not found\n", "text/plain; charset=utf-8")

            def do_POST(self) -> None:  # noqa: N802 (stdlib API)
                path = self.path.split("?", 1)[0]
                if path == "/api/stream/mode":
                    if owner.stream_control is None:
                        self._json(503, {"error": "图传模式控制未初始化"})
                        return
                    try:
                        value = self._read_json(4096)
                        result = owner.stream_control.set_mode(
                            str(value.get("mode", "")))
                        self._json(200, {"ok": True, **result})
                    except (ValueError, TypeError, json.JSONDecodeError,
                            RuntimeError, OSError) as error:
                        self._json(400, {"error": str(error)})
                    return
                if path == "/api/cameras/capture":
                    if owner.capture_client is None:
                        self._json(503, {"error": "四路摄像头抓拍未初始化"})
                        return
                    try:
                        self._json(200, owner.capture_client.capture())
                    except RuntimeError as error:
                        self._json(400, {"error": str(error)})
                    return
                if owner.io_client is None:
                    self._json(503, {"error": "语音控制未初始化"})
                    return
                try:
                    value = self._read_json(131072)
                    if path == "/api/vehicle-io/config":
                        result = owner.io_client.apply(value.get("config"))
                        self._json(200, {"ok": True,
                                         "config": result.get("config")})
                    elif path == "/api/vehicle-io/test":
                        owner.io_client.preview(value.get("config"),
                                                str(value.get("event", "")))
                        self._json(202, {"ok": True})
                    elif path == "/api/vehicle-io/mute":
                        if not isinstance(value.get("muted"), bool):
                            raise ValueError("静音值无效")
                        result = owner.io_client.set_muted(value["muted"])
                        self._json(200, {"ok": True,
                                         "config": result.get("config")})
                    elif path == "/api/vehicle-io/speaker-test":
                        owner.io_client.speaker_test(value.get("config"))
                        self._json(202, {"ok": True})
                    else:
                        self._json(404, {"error": "not found"})
                except (ValueError, TypeError, json.JSONDecodeError,
                        RuntimeError) as error:
                    self._json(400, {"error": str(error)})

            def _read_json(self, maximum: int) -> Any:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > maximum:
                    raise ValueError("请求大小无效")
                return json.loads(self.rfile.read(length).decode("utf-8"))

            def _mjpeg(self, source: str, fps: float) -> None:
                boundary = "w102frame"
                self.send_response(200)
                self.send_header("Content-Type",
                                 f"multipart/x-mixed-replace; boundary={boundary}")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                last_frame_id: Optional[int] = None
                interval = 1.0 / fps
                next_send = 0.0
                try:
                    while True:
                        frame, frame_id = owner.frame_provider(source)
                        now = time.monotonic()
                        if (frame is not None and frame_id != last_frame_id and
                                now >= next_send):
                            self.wfile.write(
                                f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                                f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                            last_frame_id = frame_id
                            next_send = (now + interval if next_send == 0.0 else
                                         next_send + interval)
                            if next_send < now:
                                next_send = now
                        time.sleep(0.002)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return

            def _json(self, status: int, value: Any) -> None:
                body = json.dumps(json_safe(value), ensure_ascii=False,
                                  separators=(",", ":")).encode("utf-8")
                self._send(status, body, "application/json; charset=utf-8")

            def _send(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Security-Policy",
                                 "default-src 'self'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; img-src 'self'")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, _format: str, *_args: Any) -> None:
                return

        try:
            self.server = ReusableThreadingHTTPServer((self.bind_host, self.port), Handler)
            self.ready_event.set()
            self.server.serve_forever(poll_interval=0.2)
        except BaseException as error:
            self.error = error
            self.ready_event.set()
        finally:
            if self.server is not None:
                self.server.server_close()

    def stop(self) -> None:
        if self.server is not None:
            self.server.shutdown()
