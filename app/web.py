"""Browser bench, calibration, and inspection-artifact viewer for the scanner."""

import copy
import hashlib
import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, abort, jsonify, render_template_string, request, send_from_directory

ROOT = Path(__file__).resolve().parents[1]
CAPTURE_DIR = ROOT / "captures"
LAYOUT_PATH = ROOT / "config" / "inspection_layout.json"
BENCH_PATH = ROOT / "config" / "bench.json"
CAPTURE_DIR.mkdir(exist_ok=True)

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drive-Over Scanner</title><style>
:root{color-scheme:dark;--bg:#101413;--panel:#18201e;--line:#31403b;--ink:#e8eee8;--muted:#a6b3aa;--accent:#d6f27d;--blue:#8bd5ff;--alert:#ff7169}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#22322c,var(--bg) 42rem);color:var(--ink);font:15px/1.45 system-ui,sans-serif}main{max-width:1440px;margin:auto;padding:28px}h1{font-size:clamp(1.7rem,4vw,3rem);margin:0}h2{font-size:.9rem;text-transform:uppercase;letter-spacing:.09em;color:var(--muted)}header{display:flex;justify-content:space-between;gap:16px;align-items:end;border-bottom:1px solid var(--line);padding-bottom:20px}.mode{color:var(--accent);font-weight:700;text-transform:uppercase;letter-spacing:.12em}.notice{margin:18px 0;padding:12px 14px;background:#25251a;border-left:3px solid #ffca68;color:#f9e7bb}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:16px}.panel{background:color-mix(in srgb,var(--panel),transparent 7%);border:1px solid var(--line);padding:16px;border-radius:8px}.feed{width:100%;aspect-ratio:16/9;object-fit:contain;background:#090c0b;border:1px solid #27332f}.thermal{image-rendering:pixelated}.controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:18px 0}button{border:0;border-radius:4px;background:var(--accent);color:#152014;padding:10px 14px;font-weight:750;cursor:pointer}button.secondary{background:#2a3933;color:var(--ink)}button.secondary.active,.layer.active{background:var(--blue);color:#10212b}button:disabled{opacity:.45;cursor:not-allowed}.status{color:var(--muted)}.radar{border:1px solid var(--line);padding:8px 10px;border-radius:4px}.radar.active{border-color:var(--alert);color:var(--alert)}.scans{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:16px}.scan{border:1px solid var(--line);padding:10px;border-radius:7px}.scan img{width:100%;background:#090c0b}.scan a{color:var(--accent);text-decoration:none}.calibration{display:none}.calibration.active{display:block}select,input,textarea{background:#0c100f;color:var(--ink);border:1px solid var(--line);padding:7px}textarea{width:100%;min-height:120px;font:12px ui-monospace,monospace}.workspace{display:grid;grid-template-columns:minmax(0,1fr) 290px;gap:16px}.canvas-wrap{background:#070908;border:1px solid var(--line);overflow:auto;max-height:70vh}.canvas-wrap canvas{display:block;margin:auto;touch-action:none;cursor:crosshair}.layers{display:flex;flex-wrap:wrap;gap:7px;margin:0 0 12px}.layer{background:#27332f;color:var(--ink);border:1px solid var(--line);padding:6px 8px;font-size:.78rem}.readout{font:12px ui-monospace,monospace;color:var(--blue);white-space:pre-wrap}.help{color:var(--muted)}.controls label{display:grid;gap:4px;font-size:.85rem}@media(max-width:750px){main{padding:16px}header{align-items:start;flex-direction:column}.workspace{grid-template-columns:1fr}}
</style></head><body><main>
<header><div><div class="mode" id="mode"></div><h1>Undercarriage Scan Console</h1></div><div class="status" id="status">Connecting...</div></header>
<div class="notice" id="notice"></div>
<section id="bench"><div class="controls"><div class="radar" id="radarIndicator">Radar: not wired</div><button id="snapshot">Save bench snapshot</button><button class="secondary" id="toggle">Open corrected layer canvas</button></div><div class="grid" id="feeds"></div>
<div class="panel calibration" id="calibration"><h2>Corrected multi-layer canvas</h2><p class="help">Every available source is rectified with its configured lens calibration before rendering. Click a source directly in the canvas or select its layer pill; drag to move only that layer. The shared canvas has the same coordinate system as deployed capture. Save All writes every available layer transform atomically but does not mark the layout calibrated.</p><div class="workspace"><div><div class="layers" id="layers"></div><div class="canvas-wrap"><canvas id="mosaicCanvas"></canvas></div></div><aside><h2 id="selectedTitle">Select a layer</h2><div class="controls"><label>Opacity <input id="alpha" type="range" min="0.1" max="1" value="0.6" step="0.05"></label><label>Scale <input id="scale" type="range" min="0.5" max="1.8" value="1" step="0.005"></label><label>Rotation <input id="rotation" type="range" min="-30" max="30" value="0" step="0.1"></label></div><div class="controls"><button class="secondary" id="flipHorizontal">Flip horizontal</button><button class="secondary" id="flipVertical">Flip vertical</button></div><div class="controls"><button class="secondary" id="resetLayer">Reset layer</button><button id="saveAll">Save all layers</button></div><div class="readout" id="transformReadout"></div><p class="help">A source marked <code>raw</code> has no lens coefficients yet. Use point anchoring to establish the first layer's position, then refine all layers here.</p></aside></div><details><summary>Anchor selected layer with matched points</summary><p class="help">Use four or more corresponding corrected-source and shared-canvas points to establish an initial transform.</p><textarea id="points" placeholder='{"source_points":[[x,y],[x,y],[x,y],[x,y]],"canvas_points":[[x,y],[x,y],[x,y],[x,y]]}'></textarea><button id="savePoints">Calculate and save selected anchor</button><span class="status" id="calibrationStatus"></span></details></div></section>
<section><h2>Technician inspections</h2><div class="scans" id="scans"></div></section>
</main><script>
const mode={{ mode|tojson }};let info={},layers=[],selectedId=null,drag=null,canvasScale=1,revision=null;
const $=id=>document.querySelector('#'+id);const sourceKey=s=>`${s.pair}:${s.source}`;
async function api(path,options){const r=await fetch(path,options);const data=await r.json();if(!r.ok)throw Error(data.error||r.statusText);return data}
function feed(kind,index,label,thermal=false){return `<article class="panel"><h2>${label}</h2><img class="feed ${thermal?'thermal':''}" data-preview-kind="${kind}" data-preview-index="${index}" alt="Waiting for source"></article>`}
function identityFor(source){const [w,h]=source.native_size,c=info.canvas,scale=Math.min(c.width/w,c.height/h)*.45;return [scale,0,(c.width-w*scale)/2,0,scale,(c.height-h*scale)/2,0,0,1]}
function normalize(matrix){const z=matrix[8]||1;return matrix.map(v=>v/z)}
function multiply(a,b){const out=Array(9).fill(0);for(let row=0;row<3;row++)for(let col=0;col<3;col++)for(let k=0;k<3;k++)out[row*3+col]+=a[row*3+k]*b[k*3+col];return out}
function translate(x,y){return [1,0,x,0,1,y,0,0,1]}function around(x,y,delta){return multiply(translate(x,y),multiply(delta,translate(-x,-y)))}
function apply(matrix,x,y){const z=matrix[6]*x+matrix[7]*y+matrix[8];return [(matrix[0]*x+matrix[1]*y+matrix[2])/z,(matrix[3]*x+matrix[4]*y+matrix[5])/z]}
function inverse(m){const [a,b,c,d,e,f,g,h,i]=m,det=a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);if(Math.abs(det)<1e-9)return null;return [(e*i-f*h)/det,(c*h-b*i)/det,(b*f-c*e)/det,(f*g-d*i)/det,(a*i-c*g)/det,(c*d-a*f)/det,(d*h-e*g)/det,(b*g-a*h)/det,(a*e-b*d)/det]}
function selected(){return layers.find(l=>l.id===selectedId)}function center(layer){return apply(layer.matrix,layer.native_size[0]/2,layer.native_size[1]/2)}
function renderLayers(){const list=$('layers');list.innerHTML=layers.map(l=>`<button class="layer ${l.id===selectedId?'active':''}" data-layer="${l.id}">${l.name} ${l.rectified?'':'(raw)'}</button>`).join('');list.querySelectorAll('[data-layer]').forEach(b=>b.onclick=()=>selectLayer(b.dataset.layer))}
function configureCanvas(){const c=$('mosaicCanvas'),maxW=1200,maxH=820,scale=Math.min(maxW/info.canvas.width,maxH/info.canvas.height);canvasScale=scale;c.width=Math.max(1,Math.round(info.canvas.width*scale));c.height=Math.max(1,Math.round(info.canvas.height*scale));c.style.width=c.width+'px';c.style.height=c.height+'px'}
function renderCanvas(){const canvas=$('mosaicCanvas'),ctx=canvas.getContext('2d');ctx.fillStyle='#070908';ctx.fillRect(0,0,canvas.width,canvas.height);for(const layer of layers){if(!layer.image.complete)continue;const m=layer.matrix,s=canvasScale;ctx.save();ctx.globalAlpha=layer.id===selectedId?1:layer.opacity;ctx.setTransform(s*m[0],s*m[3],s*m[1],s*m[4],s*m[2],s*m[5]);ctx.drawImage(layer.image,0,0,layer.native_size[0],layer.native_size[1]);if(layer.id===selectedId){ctx.strokeStyle='#d6f27d';ctx.lineWidth=2/s;ctx.strokeRect(0,0,layer.native_size[0],layer.native_size[1])}ctx.restore()}const layer=selected();$('selectedTitle').textContent=layer?layer.name:'Select a layer';if(layer){const m=layer.matrix;$('transformReadout').textContent=`source: ${layer.id}\nrectified: ${layer.rectified?'yes':'no'}\nmatrix:\n${m.slice(0,3).map(v=>v.toFixed(4)).join('  ')}\n${m.slice(3,6).map(v=>v.toFixed(4)).join('  ')}\n${m.slice(6,9).map(v=>v.toFixed(4)).join('  ')}`}else $('transformReadout').textContent=''}
function selectLayer(id){selectedId=id;const layer=selected();$('alpha').value=layer.opacity;$('scale').value=1;$('rotation').value=0;renderLayers();renderCanvas()}
function refreshLayerImage(layer){return fetch(`/api/calibration/preview/${layer.pair}/${layer.source}.jpg`,{cache:'no-store'}).then(r=>{if(!r.ok)throw Error('Preview HTTP '+r.status);return r.blob()}).then(blob=>{if(layer.url)URL.revokeObjectURL(layer.url);layer.url=URL.createObjectURL(blob);layer.image.src=layer.url}).catch(error=>{$('calibrationStatus').textContent=`${layer.name}: ${error.message}`})}
function setupLayers(){layers=info.sources.filter(s=>s.available).map(s=>{const image=new Image();const matrix=s.matrix?normalize(s.matrix):identityFor(s);const layer={...s,id:sourceKey(s),matrix,image,opacity:.6,url:null};image.onload=renderCanvas;return layer});selectedId=layers[0]?.id||null;configureCanvas();renderLayers();renderCanvas();layers.forEach(refreshLayerImage)}
function applyDelta(delta){const layer=selected();if(!layer)return;const [x,y]=center(layer);layer.matrix=normalize(multiply(around(x,y,delta),layer.matrix));renderCanvas()}
function preview(kind,index){return kind==='visible'?`/api/preview/visible.jpg?source_id=${encodeURIComponent(index)}`:`/api/preview/thermal/${index}.jpg`}
function renderRadar(radar){const e=$('radarIndicator');e.textContent=radar.enabled?(radar.detected?'Radar: vehicle detected':'Radar: clear'):'Radar: not wired';e.classList.toggle('active',Boolean(radar.detected))}
async function status(initial=false){try{info=await api('/api/status');revision=info.revision;$('mode').textContent=info.mode;$('status').textContent=info.message;$('notice').textContent=info.calibrated?'Calibration is marked active. Review artifacts and coverage before unattended operation.':'Bench mode is safe for uncalibrated hardware. Deployed capture refuses to emit uncalibrated inspection images.';renderRadar(info.radar);if(mode==='bench'&&initial){$('feeds').innerHTML=info.visible.map(s=>feed('visible',s.source_id,s.name)).join('')+info.thermal.map((s,i)=>feed('thermal',i,s.name+' thermal',true)).join('');setupLayers();refreshPreviews()}else if(mode!=='bench')$('bench').hidden=true}catch(e){$('status').textContent=e.message}}
async function scans(){try{const data=await api('/api/scans');$('scans').innerHTML=data.scans.length?data.scans.map(s=>`<article class="scan"><a href="/captures/${s.image}" target="_blank"><img src="/captures/${s.image}" loading="lazy"><strong>${s.event_id}</strong></a><div class="status">${s.stats}</div></article>`).join(''):'<p class="status">No deployed inspection mosaics yet.</p>'}catch(e){$('scans').textContent=e.message}}
async function refreshPreview(image){if(image.dataset.loading)return;image.dataset.loading='1';try{const r=await fetch(preview(image.dataset.previewKind,image.dataset.previewIndex),{cache:'no-store'});if(!r.ok)throw Error(`Preview HTTP ${r.status}`);const url=URL.createObjectURL(await r.blob()),previous=image.dataset.objectUrl;image.src=url;image.dataset.objectUrl=url;if(previous)URL.revokeObjectURL(previous)}catch(e){image.alt=e.message}finally{delete image.dataset.loading}}
function refreshPreviews(){document.querySelectorAll('.feed[data-preview-kind]').forEach(refreshPreview)}
$('snapshot').onclick=async()=>{try{const r=await api('/api/snapshot',{method:'POST'});$('status').textContent='Saved '+r.event_id}catch(e){alert(e.message)}};$('toggle').onclick=()=>{$('calibration').classList.toggle('active');renderCanvas()};
$('mosaicCanvas').addEventListener('pointerdown',e=>{const box=e.currentTarget.getBoundingClientRect(),x=(e.clientX-box.left)/canvasScale,y=(e.clientY-box.top)/canvasScale;for(const layer of [...layers].reverse()){const inv=inverse(layer.matrix);if(!inv)continue;const [sx,sy]=apply(inv,x,y);if(sx>=0&&sy>=0&&sx<=layer.native_size[0]&&sy<=layer.native_size[1]){selectLayer(layer.id);drag={x,y};e.currentTarget.setPointerCapture(e.pointerId);break}}});$('mosaicCanvas').addEventListener('pointermove',e=>{if(!drag)return;const box=e.currentTarget.getBoundingClientRect(),x=(e.clientX-box.left)/canvasScale,y=(e.clientY-box.top)/canvasScale,layer=selected();layer.matrix=normalize(multiply(translate(x-drag.x,y-drag.y),layer.matrix));drag={x,y};renderCanvas()});for(const event of ['pointerup','pointercancel','pointerleave'])$('mosaicCanvas').addEventListener(event,()=>drag=null);
$('alpha').oninput=()=>{const layer=selected();if(layer){layer.opacity=+$('alpha').value;renderCanvas()}};let controlScale=1,controlRotation=0;$('scale').oninput=()=>{const value=+$('scale').value;applyDelta([value/controlScale,0,0,0,value/controlScale,0,0,0,1]);controlScale=value};$('rotation').oninput=()=>{const next=+$('rotation').value,delta=(next-controlRotation)*Math.PI/180,c=Math.cos(delta),s=Math.sin(delta);applyDelta([c,-s,0,s,c,0,0,0,1]);controlRotation=next};$('flipHorizontal').onclick=()=>applyDelta([-1,0,0,0,1,0,0,0,1]);$('flipVertical').onclick=()=>applyDelta([1,0,0,0,-1,0,0,0,1]);$('resetLayer').onclick=()=>{const layer=selected();if(layer){layer.matrix=identityFor(layer);controlScale=1;controlRotation=0;$('scale').value=1;$('rotation').value=0;renderCanvas()}};
$('saveAll').onclick=async()=>{try{const r=await api('/api/calibration/transforms',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({revision,transforms:layers.map(l=>({pair:l.pair,source:l.source,matrix:l.matrix}))})});revision=r.revision;$('calibrationStatus').textContent=`Saved ${r.count} layer transforms. Layout remains uncalibrated.`}catch(e){$('calibrationStatus').textContent=e.message}};$('savePoints').onclick=async()=>{const layer=selected();if(!layer)return;try{const body=JSON.parse($('points').value);body.pair=layer.pair;body.source=layer.source;const r=await api('/api/calibration/homography',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});layer.matrix=normalize(r.transform);revision=r.revision;$('calibrationStatus').textContent=`Anchor saved. RMS reprojection error: ${r.rms.toFixed(2)} px`;renderCanvas()}catch(e){$('calibrationStatus').textContent=e.message}};
status(true);scans();setInterval(scans,10000);setInterval(()=>{if(mode==='bench'){refreshPreviews();layers.forEach(refreshLayerImage);status(false)}},1000);
</script></body></html>"""


class BenchHardware:
    def __init__(self, config):
        self.lock = threading.Lock()
        self.cameras = {}
        self.thermals = []
        self.config = config
        self.preview_sequence = 0
        self.radar = None
        self.radar_active_high = True

    def start(self):
        import adafruit_mlx90640
        import board
        import busio
        from adafruit_tca9548a import TCA9548A
        from gpiozero import DigitalInputDevice
        from picamera2 import Picamera2

        try:
            camera_ids = set()
            source_names = set()
            for source in self.config["visible_sources"]:
                visible = parse_bench_visible_source(source)
                if visible["id"] in camera_ids or source["name"] in source_names:
                    raise ValueError("Bench visible source IDs and names must be unique")
                camera_ids.add(visible["id"])
                source_names.add(source["name"])
                if visible["kind"] == "picamera2":
                    camera = Picamera2(visible["index"])
                    camera.configure(camera.create_still_configuration(main={"size": visible["size"], "format": "RGB888"}))
                    camera.start()
                else:
                    camera = cv2.VideoCapture(visible["device"], cv2.CAP_V4L2)
                    camera.set(cv2.CAP_PROP_FRAME_WIDTH, visible["size"][0])
                    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, visible["size"][1])
                    if not camera.isOpened():
                        raise RuntimeError(f"Could not open {visible['device']}")
                self.cameras[visible["id"]] = (source, visible, camera)
            i2c = busio.I2C(board.SCL, board.SDA)
            multiplexer = None
            thermal_channels = set()
            for source in self.config["thermal_sources"]:
                if source["name"] in source_names:
                    raise ValueError("Bench source names must be unique")
                source_names.add(source["name"])
                if source["kind"] == "direct":
                    bus = i2c
                elif source["kind"] == "tca9548a":
                    if source["channel"] in thermal_channels:
                        raise ValueError("Bench TCA9548A channels must be unique")
                    thermal_channels.add(source["channel"])
                    multiplexer = multiplexer or TCA9548A(i2c)
                    bus = multiplexer[source["channel"]]
                else:
                    raise ValueError("Thermal source kind must be direct or tca9548a")
                thermal = adafruit_mlx90640.MLX90640(bus)
                thermal.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
                self.thermals.append((source, thermal))
            radar = self.config.get("radar", {})
            if radar.get("enabled", False):
                self.radar = DigitalInputDevice(radar.get("bcm_pin", 17), pull_up=False)
                self.radar_active_high = radar.get("active_high", True)
        except Exception:
            self.close()
            raise

    def close(self):
        for _source, visible, camera in self.cameras.values():
            if visible["kind"] == "picamera2":
                camera.close()
            else:
                camera.release()
        self.cameras.clear()
        if self.radar is not None:
            self.radar.close()
            self.radar = None

    def visible(self, source_id, stamp=False):
        with self.lock:
            if source_id not in self.cameras:
                raise KeyError(source_id)
            source, visible, camera = self.cameras[source_id]
            if visible["kind"] == "picamera2":
                frame = cv2.cvtColor(camera.capture_array("main"), cv2.COLOR_RGB2BGR)
            else:
                success, frame = camera.read()
                if not success or frame is None:
                    raise RuntimeError(f"V4L2 capture failed for {visible['device']}")
            return self._stamp(frame, f"VISIBLE {source['name']}") if stamp else frame

    def thermal(self, index):
        with self.lock:
            source, sensor = self.thermals[index]
            frame = np.zeros(768, dtype=np.float32)
            sensor.getFrame(frame)
            return source, frame.reshape(24, 32)

    def radar_status(self):
        if self.radar is None:
            return {"enabled": False, "detected": False}
        with self.lock:
            value = bool(self.radar.value)
        return {"enabled": True, "detected": value if self.radar_active_high else not value}

    def stamp(self, image, label):
        with self.lock:
            return self._stamp(image, label)

    def _stamp(self, image, label):
        self.preview_sequence += 1
        cv2.rectangle(image, (0, 0), (360, 40), (0, 0, 0), -1)
        cv2.putText(image, f"{label}  #{self.preview_sequence}  {time.monotonic():.3f}", (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        return image


def load_json(path):
    with path.open(encoding="utf-8") as source:
        return json.load(source)


def layout_revision(layout):
    payload = json.dumps(layout, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(payload).hexdigest()[:16]


def save_layout(layout):
    temporary = LAYOUT_PATH.with_suffix(".json.tmp")
    with temporary.open("w", encoding="utf-8") as destination:
        json.dump(layout, destination, indent=2, allow_nan=False)
        destination.flush()
        os.fsync(destination.fileno())
    os.replace(temporary, LAYOUT_PATH)


def matrix_or_none(values):
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-9:
        return None
    return matrix / matrix[2, 2]


def jpeg(image):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError("Could not encode preview")
    return encoded.tobytes()


def image_response(image, rectified=None):
    response = Response(jpeg(image), mimetype="image/jpeg")
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    if rectified is not None:
        response.headers["X-Scanner-Rectified"] = str(bool(rectified)).lower()
    return response


def thermal_preview(frame, thermal_range):
    low, high = thermal_range
    normalized = np.uint8(np.clip((frame - low) / (high - low), 0, 1) * 255)
    image = cv2.applyColorMap(normalized, cv2.COLORMAP_INFERNO)
    image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_NEAREST)
    cv2.putText(image, f"{np.nanmin(frame):.1f} to {np.nanmax(frame):.1f} C", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return image


def source_key(source):
    return f"{source}_to_canvas"


def parse_bench_visible_source(source):
    if source.get("kind") not in {"picamera2", "v4l2"}:
        raise ValueError("Bench visible source kind must be picamera2 or v4l2")
    size = source.get("size")
    if not isinstance(size, list) or len(size) != 2 or any(not isinstance(value, int) or value <= 0 for value in size):
        raise ValueError("Bench visible source size must contain positive width and height")
    visible = {"kind": source["kind"], "size": tuple(size)}
    if visible["kind"] == "picamera2":
        if not isinstance(source.get("index"), int) or source["index"] < 0:
            raise ValueError("Bench Picamera2 source requires a non-negative index")
        visible["index"] = source["index"]
        visible["id"] = f"picamera2:{source['index']}"
    else:
        device = source.get("device")
        if not isinstance(device, str) or not device.startswith("/dev/"):
            raise ValueError("Bench V4L2 source requires an absolute /dev device path")
        visible["device"] = device
        visible["id"] = f"v4l2:{device}"
    return visible


def source_size(source):
    return [2304, 1296] if source == "visible" else [32, 24]


def rectify(image, pair, source):
    matrix = matrix_or_none(pair.get(f"{source}_camera_matrix"))
    distortion = np.asarray(pair.get(f"{source}_distortion_coefficients"), dtype=np.float64).reshape(-1)
    if matrix is None or distortion.size < 4 or not np.isfinite(distortion).all():
        return image, False
    return cv2.undistort(image, matrix, distortion), True


def create_app(mode):
    app = Flask(__name__)
    layout = load_json(LAYOUT_PATH)
    layout_lock = threading.Lock()
    bench = None
    startup_error = None
    if mode == "bench":
        try:
            bench = BenchHardware(load_json(BENCH_PATH))
            bench.start()
        except Exception as error:
            startup_error = str(error)

    def pair_by_name(name):
        return next((pair for pair in layout.get("pairs", []) if pair["name"] == name), None)

    def thermal_indexes():
        return {source.get("pair_name"): index for index, (source, _sensor) in enumerate(bench.thermals) if source.get("pair_name")} if bench else {}

    def visible_indexes():
        return {
            source.get("pair_name"): visible["id"]
            for source, visible, _camera in bench.cameras.values()
            if source.get("pair_name")
        } if bench else {}

    def available_sources():
        thermal_by_pair = thermal_indexes()
        visible_by_pair = visible_indexes()
        sources = []
        for pair in layout.get("pairs", []):
            visible_index = visible_by_pair.get(pair["name"])
            for source, index in (("visible", visible_index), ("thermal", thermal_by_pair.get(pair["name"]))):
                available = (source == "visible" and bench is not None and index in bench.cameras) or (source == "thermal" and index is not None)
                transform = pair.get(source_key(source))
                sources.append({
                    "id": f"{pair['name']}:{source}",
                    "pair": pair["name"],
                    "source": source,
                    "name": f"{pair['name']} {source}",
                    "available": available,
                    "index": index,
                    "native_size": source_size(source),
                    "matrix": transform,
                    "rectified": matrix_or_none(pair.get(f"{source}_camera_matrix")) is not None,
                })
        return sources

    def calibration_image(pair, source):
        if bench is None:
            raise RuntimeError(startup_error or "Bench hardware is unavailable")
        source_info = next((item for item in available_sources() if item["pair"] == pair["name"] and item["source"] == source), None)
        if source_info is None or not source_info["available"]:
            raise KeyError(source)
        if source == "visible":
            image, rectified = rectify(bench.visible(source_info["index"]), pair, source)
            return image, rectified
        _thermal_source, temperatures = bench.thermal(source_info["index"])
        temperatures, rectified = rectify(temperatures, pair, source)
        low, high = layout["thermal_range_c"]
        heatmap = cv2.applyColorMap(np.uint8(np.clip((temperatures - low) / (high - low), 0, 1) * 255), cv2.COLORMAP_INFERNO)
        return heatmap, rectified

    @app.get("/")
    def index():
        return render_template_string(PAGE, mode=mode)

    @app.errorhandler(404)
    @app.errorhandler(409)
    @app.errorhandler(502)
    @app.errorhandler(503)
    def api_error(error):
        if request.path.startswith("/api/"):
            return jsonify(error=error.description), error.code
        return error

    @app.get("/api/status")
    def status():
        with layout_lock:
            return jsonify({
                "mode": mode,
                "message": startup_error or ("Hardware preview ready" if bench else "Inspection artifact viewer ready"),
                "calibrated": bool(layout.get("calibrated")),
                "revision": layout_revision(layout),
                "canvas": layout["canvas"],
                "visible": [
                    {**source, "source_id": visible["id"]}
                    for source, visible, _camera in bench.cameras.values()
                ] if bench else [],
                "thermal": [source for source, _sensor in bench.thermals] if bench else [],
                "radar": bench.radar_status() if bench else {"enabled": False, "detected": False},
                "sources": available_sources(),
            })

    @app.get("/api/preview/visible.jpg")
    def visible_preview():
        if bench is None:
            abort(503, startup_error or "Bench hardware is unavailable")
        try:
            source_id = request.args.get("source_id", "")
            return image_response(cv2.resize(bench.visible(source_id, stamp=True), (960, 540)))
        except KeyError:
            abort(404)

    @app.get("/api/preview/thermal/<int:index>.jpg")
    def thermal_preview_route(index):
        if bench is None:
            abort(503, startup_error or "Bench hardware is unavailable")
        try:
            source, frame = bench.thermal(index)
            return image_response(bench.stamp(thermal_preview(frame, layout["thermal_range_c"]), f"THERMAL {source['name']}"))
        except IndexError:
            abort(404)
        except Exception as error:
            abort(502, f"Thermal read failed: {error}")

    @app.get("/api/calibration/preview/<pair_name>/<source>.jpg")
    def calibration_preview(pair_name, source):
        if source not in {"visible", "thermal"}:
            abort(404)
        with layout_lock:
            pair = pair_by_name(pair_name)
            if pair is None:
                abort(404)
            try:
                image, rectified = calibration_image(pair, source)
                return image_response(image, rectified)
            except KeyError:
                abort(404, f"{pair_name} {source} is not available in bench mode")
            except Exception as error:
                abort(502, f"Corrected preview failed: {error}")

    @app.post("/api/snapshot")
    def snapshot():
        if bench is None:
            return jsonify(error=startup_error or "Bench hardware is unavailable"), 503
        event_id = datetime.now().strftime("bench_%Y%m%d_%H%M%S_%f")
        try:
            for source_id, (source, _visible, _camera) in bench.cameras.items():
                cv2.imwrite(str(CAPTURE_DIR / f"{event_id}_{source['name']}.jpg"), bench.visible(source_id))
            for index in range(len(bench.thermals)):
                source, frame = bench.thermal(index)
                np.save(CAPTURE_DIR / f"{event_id}_{source['name']}.npy", frame)
                cv2.imwrite(str(CAPTURE_DIR / f"{event_id}_{source['name']}.png"), thermal_preview(frame, layout["thermal_range_c"]))
        except Exception as error:
            return jsonify(error=f"Bench snapshot failed: {error}"), 502
        return jsonify(event_id=event_id)

    @app.post("/api/calibration/homography")
    def save_homography():
        nonlocal layout
        body = request.get_json(silent=True) or {}
        source = body.get("source")
        source_points = np.asarray(body.get("source_points"), dtype=np.float32)
        canvas_points = np.asarray(body.get("canvas_points"), dtype=np.float32)
        if source not in {"visible", "thermal"} or source_points.shape != canvas_points.shape or source_points.ndim != 2 or source_points.shape[0] < 4 or source_points.shape[1] != 2:
            return jsonify(error="Provide a known source and four or more [x, y] point pairs"), 400
        homography, mask = cv2.findHomography(source_points, canvas_points, cv2.RANSAC)
        matrix = matrix_or_none(homography)
        if matrix is None or int(mask.sum()) < 4:
            return jsonify(error="Points do not produce a valid homography"), 400
        projected = cv2.perspectiveTransform(source_points[None, :, :], matrix.astype(np.float32))[0]
        rms = float(np.sqrt(np.mean(np.sum((projected - canvas_points) ** 2, axis=1))))
        with layout_lock:
            pair = pair_by_name(body.get("pair"))
            if pair is None:
                return jsonify(error="Select a known pair"), 400
            candidate = copy.deepcopy(layout)
            next_pair = next(item for item in candidate["pairs"] if item["name"] == pair["name"])
            next_pair[source_key(source)] = matrix.tolist()
            candidate["calibrated"] = False
            save_layout(candidate)
            layout = candidate
            return jsonify(rms=rms, transform=matrix.tolist(), revision=layout_revision(layout))

    @app.post("/api/calibration/transforms")
    def save_transforms():
        nonlocal layout
        body = request.get_json(silent=True) or {}
        records = body.get("transforms")
        if not isinstance(records, list) or not records:
            return jsonify(error="Provide one or more layer transforms"), 400
        with layout_lock:
            if body.get("revision") != layout_revision(layout):
                return jsonify(error="Layout changed in another session; reload before saving"), 409
            candidate = copy.deepcopy(layout)
            known = {(pair["name"], source) for pair in candidate["pairs"] for source in ("visible", "thermal")}
            seen = set()
            for record in records:
                key = (record.get("pair"), record.get("source"))
                matrix = matrix_or_none(record.get("matrix"))
                if key not in known or key in seen or matrix is None:
                    return jsonify(error="Every transform must name a unique known pair/source with a valid 3x3 matrix"), 400
                seen.add(key)
                next_pair = next(pair for pair in candidate["pairs"] if pair["name"] == key[0])
                next_pair[source_key(key[1])] = matrix.tolist()
            candidate["calibrated"] = False
            save_layout(candidate)
            layout = candidate
            return jsonify(count=len(records), revision=layout_revision(layout))

    @app.get("/api/scans")
    def scans():
        results = []
        for image in sorted(CAPTURE_DIR.glob("*_inspection.jpg"), reverse=True)[:24]:
            event_id = image.name.removesuffix("_inspection.jpg")
            metadata_path = CAPTURE_DIR / f"{event_id}_metadata.json"
            stats = "No metadata"
            if metadata_path.exists():
                metadata = load_json(metadata_path)
                stats = f"{metadata.get('maximum_speed_mps', 0):.2f} m/s max | {metadata.get('thermal_coverage', 0):.0%} thermal coverage"
            results.append({"event_id": event_id, "image": image.name, "stats": stats})
        return jsonify(scans=results)

    @app.get("/captures/<path:name>")
    def captures(name):
        return send_from_directory(CAPTURE_DIR, name)

    return app


def main(mode):
    app = create_app(mode)
    app.run(host="0.0.0.0", port=int(os.environ.get("SCANNER_WEB_PORT", "8080")), threaded=True)
