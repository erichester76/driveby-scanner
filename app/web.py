"""Browser bench, calibration, and inspection-artifact viewer for the scanner."""

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
:root{color-scheme:dark;--bg:#101413;--panel:#18201e;--line:#31403b;--ink:#e8eee8;--muted:#a6b3aa;--accent:#d6f27d;--warning:#ffca68;--blue:#8bd5ff}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#22322c,var(--bg) 42rem);color:var(--ink);font:15px/1.45 system-ui,sans-serif}main{max-width:1440px;margin:auto;padding:28px}h1{font-size:clamp(1.7rem,4vw,3rem);margin:0}h2{font-size:1rem;text-transform:uppercase;letter-spacing:.09em;color:var(--muted)}header{display:flex;justify-content:space-between;gap:16px;align-items:end;border-bottom:1px solid var(--line);padding-bottom:20px}.mode{color:var(--accent);font-weight:700;text-transform:uppercase;letter-spacing:.12em}.notice{margin:18px 0;padding:12px 14px;background:#25251a;border-left:3px solid var(--warning);color:#f9e7bb}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:16px}.panel{background:color-mix(in srgb,var(--panel),transparent 7%);border:1px solid var(--line);padding:16px;border-radius:8px}.feed{width:100%;aspect-ratio:16/9;object-fit:contain;background:#090c0b;border:1px solid #27332f}.thermal{image-rendering:pixelated}.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:18px 0}button{border:0;border-radius:4px;background:var(--accent);color:#152014;padding:11px 15px;font-weight:750;cursor:pointer}button.secondary{background:#2a3933;color:var(--ink)}button.secondary.active{background:var(--blue);color:#10212b}button:disabled{opacity:.45;cursor:not-allowed}.status{color:var(--muted)}.scans{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:16px}.scan{border:1px solid var(--line);padding:10px;border-radius:7px}.scan img{width:100%;background:#090c0b}.scan a{color:var(--accent);text-decoration:none}.calibration{display:none}.calibration.active{display:block}textarea{width:100%;min-height:125px;background:#0c100f;color:var(--ink);border:1px solid var(--line);padding:10px;font:12px ui-monospace,monospace}select,input{background:#0c100f;color:var(--ink);border:1px solid var(--line);padding:7px}.registration{display:grid;grid-template-columns:minmax(0,1fr) 290px;gap:16px}.stage{position:relative;width:100%;aspect-ratio:16/9;background:#070908;border:1px solid var(--line);overflow:hidden;touch-action:none}.stage img{position:absolute;width:100%;height:100%;object-fit:fill;user-select:none;-webkit-user-drag:none}.stage #movingLayer{opacity:.5;cursor:grab;transform-origin:50% 50%;filter:saturate(1.25)}.stage #movingLayer.dragging{cursor:grabbing}.readout{font:12px ui-monospace,monospace;color:var(--blue);white-space:pre-wrap}.help{color:var(--muted);margin-top:0}.source-row{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}.legacy{margin-top:20px}.legend-dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--blue);margin-right:5px}@media(max-width:750px){main{padding:16px}header{align-items:start;flex-direction:column}.registration{grid-template-columns:1fr}.source-row{grid-template-columns:1fr}}
.stage{aspect-ratio:32/9}.stage #referenceLayer,.stage #movingLayer{left:25%;width:50%}.stage::after{content:"";position:absolute;left:25%;width:50%;height:100%;border:1px dashed rgba(214,242,125,.7);pointer-events:none}
</style></head><body><main>
<header><div><div class="mode" id="mode"></div><h1>Undercarriage Scan Console</h1></div><div class="status" id="status">Connecting...</div></header>
<div class="notice" id="notice"></div>
<section id="bench"><div class="controls"><div id="radarIndicator" class="status">Radar: not wired</div><button id="snapshot">Save bench snapshot</button><button class="secondary" id="toggle">Open registration workbench</button></div><div class="grid" id="feeds"></div>
<div class="panel calibration" id="calibration"><h2>Relative registration workbench</h2><p class="help">The dashed center frame is the anchored reference. The stage is twice as wide, so place the blue moving layer to either side with only the measured overlap to create a wider stitched mosaic. Anchor one source to the shared canvas with point mapping below, then register the other source and save.</p>
<div class="registration"><div><div class="source-row"><label>Reference pair<br><select id="referencePair"></select></label><label>Reference source<br><select id="referenceSource"><option value="visible">Visible</option><option value="thermal">Thermal</option></select></label><label>Moving target pair<br><select id="targetPair"></select></label><label>Moving source<br><select id="targetSource"><option value="visible">Visible</option><option value="thermal">Thermal</option></select></label></div><div class="stage" id="stage"><img id="referenceLayer" alt="Reference source"><img id="movingLayer" alt="Moving source"></div></div><aside><h2>Layer controls</h2><label>Blend <input id="alpha" type="range" min="0.1" max="0.9" value="0.5" step="0.05"></label><br><label>Scale <input id="scale" type="range" min="0.5" max="1.8" value="1" step="0.005"></label><br><label>Rotation <input id="rotation" type="range" min="-20" max="20" value="0" step="0.1"></label><div class="controls"><button class="secondary" id="flipHorizontal">Flip horizontal</button><button class="secondary" id="flipVertical">Flip vertical</button></div><div class="controls"><button class="secondary" id="resetTransform">Reset</button><button id="saveRelative">Save registration</button></div><div class="readout" id="transformReadout"></div><p class="help"><span class="legend-dot"></span>The moving layer is blue-tinted. Flips, scale, rotation, and translation are saved in the target calibration transform.</p></aside></div>
<details class="legacy"><summary>Anchor a source to the shared canvas with matched points</summary><p class="help">Use this only to establish a reference transform. Enter four or more matching source and shared-canvas points.</p><label>Target pair <select id="pointPair"></select></label> <label>Source <select id="pointSource"><option value="visible">Visible</option><option value="thermal">Thermal</option></select></label><textarea id="points" placeholder='{"source_points":[[x,y],[x,y],[x,y],[x,y]],"canvas_points":[[x,y],[x,y],[x,y],[x,y]]}'></textarea><button id="savePoints">Calculate and save anchor transform</button><span class="status" id="calibrationStatus"></span></details></div></section>
<section><h2>Technician inspections</h2><div class="scans" id="scans"></div></section>
</main><script>
const mode={{ mode|tojson }}, stageSize=[1920,540];let info={},drag=null,transform={x:0,y:0,scale:1,rotation:0,flipX:false,flipY:false};
const $=id=>document.querySelector('#'+id);async function api(path,options){const r=await fetch(path,options);const data=await r.json();if(!r.ok)throw Error(data.error||r.statusText);return data}
function feed(kind,index,label,thermal=false){return `<article class="panel"><h2>${label}</h2><img class="feed ${thermal?'thermal':''}" data-preview-kind="${kind}" data-preview-index="${index}" alt="Waiting for source"></article>`}
function options(select,values){select.innerHTML=values.map(v=>`<option value="${v.name}">${v.name}</option>`).join('')}
function preview(kind,index){return `/api/preview/${kind}/${index}.jpg?t=${Date.now()}`}
function pair(name){return info.pairs.find(p=>p.name===name)}function activeSource(selectPair,selectSource){const p=pair($(selectPair).value),source=$(selectSource).value;return source==='visible'?{kind:'visible',index:p.visible_camera_index}:{kind:'thermal',index:p.bench_thermal_index}}function renderRadar(radar){const indicator=$('radarIndicator');indicator.textContent=radar.enabled?(radar.detected?'Radar: vehicle detected':'Radar: clear'):'Radar: not wired';indicator.style.color=radar.detected?'#ff7169':''}
function matrix(){const r=transform.rotation*Math.PI/180,fx=transform.flipX?-1:1,fy=transform.flipY?-1:1,c=Math.cos(r)*transform.scale,s=Math.sin(r)*transform.scale,cx=stageSize[0]/2,cy=stageSize[1]/2,a=fx*c,b=-fx*s,d=fy*s,e=fy*c;return [[a,b,transform.x+cx-a*cx-b*cy],[d,e,transform.y+cy-d*cx-e*cy],[0,0,1]]}
function renderTransform(){const m=matrix();$('movingLayer').style.opacity=$('alpha').value;$('movingLayer').style.transform=`translate(${transform.x}px,${transform.y}px) scaleX(${transform.flipX?-1:1}) scaleY(${transform.flipY?-1:1}) scale(${transform.scale}) rotate(${transform.rotation}deg)`;$('flipHorizontal').classList.toggle('active',transform.flipX);$('flipVertical').classList.toggle('active',transform.flipY);$('transformReadout').textContent=`x: ${transform.x.toFixed(1)} px\ny: ${transform.y.toFixed(1)} px\nscale: ${transform.scale.toFixed(3)}\nrotation: ${transform.rotation.toFixed(1)} deg\nflip X: ${transform.flipX?'on':'off'}\nflip Y: ${transform.flipY?'on':'off'}\nM: ${m.flat().map(v=>v.toFixed(4)).join(', ')}`}
function loadLayers(){try{const ref=activeSource('referencePair','referenceSource'),target=activeSource('targetPair','targetSource');if(target.index===undefined)throw Error('This thermal source is not available in bench.json');$('referenceLayer').src=preview(ref.kind,ref.index);$('movingLayer').src=preview(target.kind,target.index);renderTransform()}catch(e){$('calibrationStatus').textContent=e.message}}
async function status(){try{info=await api('/api/status');$('mode').textContent=info.mode;$('status').textContent=info.message;$('notice').textContent=info.calibrated?'Calibration is marked active. Review artifacts and coverage before unattended operation.':'Bench mode is safe for uncalibrated hardware. Deployed capture refuses to produce inspection images until calibration is complete.';renderRadar(info.radar);if(mode==='bench'){$('feeds').innerHTML=info.visible.map(i=>feed('visible',i,'Visible camera '+i)).join('')+info.thermal.map((s,i)=>feed('thermal',i,s.name+' thermal',true)).join('');options($('referencePair'),info.pairs);options($('targetPair'),info.pairs);options($('pointPair'),info.pairs);$('referencePair').value=info.pairs[0]?.name;$('targetPair').value=info.pairs[0]?.name;loadLayers();refreshPreviews()}else $('bench').hidden=true}catch(e){$('status').textContent=e.message}}
async function scans(){try{const data=await api('/api/scans');$('scans').innerHTML=data.scans.length?data.scans.map(s=>`<article class="scan"><a href="/captures/${s.image}" target="_blank"><img src="/captures/${s.image}" loading="lazy"><strong>${s.event_id}</strong></a><div class="status">${s.stats}</div></article>`).join(''):'<p class="status">No deployed inspection mosaics yet.</p>'}catch(e){$('scans').textContent=e.message}}
$('snapshot').onclick=async()=>{try{const r=await api('/api/snapshot',{method:'POST'});$('status').textContent='Saved '+r.event_id}catch(e){alert(e.message)}};$('toggle').onclick=()=>{$('calibration').classList.toggle('active');loadLayers()};
for(const id of ['referencePair','referenceSource','targetPair','targetSource'])$(id).onchange=loadLayers;for(const id of ['alpha','scale','rotation'])$(id).oninput=()=>{transform.scale=+$('scale').value;transform.rotation=+$('rotation').value;renderTransform()};$('flipHorizontal').onclick=()=>{transform.flipX=!transform.flipX;renderTransform()};$('flipVertical').onclick=()=>{transform.flipY=!transform.flipY;renderTransform()};$('resetTransform').onclick=()=>{transform={x:0,y:0,scale:1,rotation:0,flipX:false,flipY:false};$('scale').value=1;$('rotation').value=0;renderTransform()};
$('stage').addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,startX:transform.x,startY:transform.y};$('movingLayer').classList.add('dragging');e.target.setPointerCapture?.(e.pointerId)});$('stage').addEventListener('pointermove',e=>{if(!drag)return;const box=$('stage').getBoundingClientRect();transform.x=drag.startX+(e.clientX-drag.x)*stageSize[0]/box.width;transform.y=drag.startY+(e.clientY-drag.y)*stageSize[1]/box.height;renderTransform()});for(const event of ['pointerup','pointercancel','pointerleave'])$('stage').addEventListener(event,()=>{drag=null;$('movingLayer').classList.remove('dragging')});
$('saveRelative').onclick=async()=>{try{const ref=activeSource('referencePair','referenceSource'),target=activeSource('targetPair','targetSource');const result=await api('/api/calibration/relative',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reference_pair:$('referencePair').value,reference_source:ref.kind,target_pair:$('targetPair').value,target_source:target.kind,stage_transform:matrix(),stage_size:stageSize})});$('calibrationStatus').textContent=`Saved ${result.target_key}; reference composition complete.`}catch(e){$('calibrationStatus').textContent=e.message}};
$('savePoints').onclick=async()=>{try{const body=JSON.parse($('points').value);body.pair=$('pointPair').value;body.source=$('pointSource').value;const r=await api('/api/calibration/homography',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});$('calibrationStatus').textContent='Anchor saved. RMS reprojection error: '+r.rms.toFixed(2)+' px'}catch(e){$('calibrationStatus').textContent=e.message}};
async function refreshPreview(image){if(image.dataset.loading)return;image.dataset.loading='1';try{const response=await fetch(`/api/preview/${image.dataset.previewKind}/${image.dataset.previewIndex}.jpg`,{cache:'no-store'});if(!response.ok)throw Error(`Preview HTTP ${response.status}`);const objectUrl=URL.createObjectURL(await response.blob()),previous=image.dataset.objectUrl;image.src=objectUrl;image.dataset.objectUrl=objectUrl;if(previous)URL.revokeObjectURL(previous)}catch(error){image.alt=error.message}finally{delete image.dataset.loading}}
function refreshPreviews(){document.querySelectorAll('.feed[data-preview-kind]').forEach(refreshPreview)}
async function refreshRadar(){if(mode!=='bench')return;try{renderRadar((await api('/api/status')).radar)}catch(error){$('radarIndicator').textContent='Radar: unavailable'}}
status();scans();setInterval(scans,10000);setInterval(()=>{if(mode==='bench')refreshPreviews()},1000);setInterval(refreshRadar,250);
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
            for index in self.config["visible_camera_indexes"]:
                camera = Picamera2(index)
                camera.configure(camera.create_still_configuration(main={"size": (2304, 1296), "format": "RGB888"}))
                camera.start()
                self.cameras[index] = camera
            i2c = busio.I2C(board.SCL, board.SDA)
            multiplexer = None
            for source in self.config["thermal_sources"]:
                if source["kind"] == "direct":
                    bus = i2c
                elif source["kind"] == "tca9548a":
                    multiplexer = multiplexer or TCA9548A(i2c)
                    bus = multiplexer[source["channel"]]
                else:
                    raise ValueError("Thermal source kind must be direct or tca9548a")
                thermal = adafruit_mlx90640.MLX90640(bus)
                thermal.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
                self.thermals.append((source, thermal))
            radar_config = self.config.get("radar", {})
            if radar_config.get("enabled", False):
                self.radar = DigitalInputDevice(radar_config.get("bcm_pin", 17), pull_up=False)
                self.radar_active_high = radar_config.get("active_high", True)
        except Exception:
            self.close()
            raise

    def close(self):
        for camera in self.cameras.values():
            camera.close()
        self.cameras.clear()
        if self.radar is not None:
            self.radar.close()
            self.radar = None

    def radar_status(self):
        if self.radar is None:
            return {"enabled": False, "detected": False}
        with self.lock:
            detected = bool(self.radar.value)
        return {"enabled": True, "detected": detected if self.radar_active_high else not detected}

    def visible(self, index):
        with self.lock:
            if index not in self.cameras:
                raise KeyError(index)
            frame = cv2.cvtColor(self.cameras[index].capture_array("main"), cv2.COLOR_RGB2BGR)
            return self._stamp(frame, f"VISIBLE {index}")

    def thermal(self, source_index):
        with self.lock:
            source, sensor = self.thermals[source_index]
            frame = np.zeros(24 * 32, dtype=np.float32)
            sensor.getFrame(frame)
            return source, frame.reshape(24, 32)

    def stamp_thermal(self, image, name):
        with self.lock:
            return self._stamp(image, f"THERMAL {name}")

    def _stamp(self, image, label):
        self.preview_sequence += 1
        timestamp = time.monotonic()
        cv2.rectangle(image, (0, 0), (360, 40), (0, 0, 0), -1)
        cv2.putText(
            image,
            f"{label}  #{self.preview_sequence}  {timestamp:.3f}",
            (10, 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return image


def load_json(path):
    with path.open(encoding="utf-8") as source:
        return json.load(source)


def jpeg(image, quality=85):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("Could not encode preview")
    return encoded.tobytes()


def preview_response(image):
    response = Response(jpeg(image), mimetype="image/jpeg")
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response


def thermal_preview(frame, thermal_range):
    low, high = thermal_range
    normalized = np.uint8(np.clip((frame - low) / (high - low), 0, 1) * 255)
    image = cv2.applyColorMap(normalized, cv2.COLORMAP_INFERNO)
    image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_NEAREST)
    cv2.putText(image, f"{np.nanmin(frame):.1f} to {np.nanmax(frame):.1f} C", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return image


def source_size(source):
    return (2304, 1296) if source == "visible" else (32, 24)


def to_canvas_key(source):
    return f"{source}_to_canvas"


def source_to_stage(source, stage_size):
    width, height = source_size(source)
    slot_width = stage_size[0] / 2
    return np.array([
        [slot_width / width, 0, (stage_size[0] - slot_width) / 2],
        [0, stage_size[1] / height, 0],
        [0, 0, 1],
    ], dtype=np.float64)


def create_app(mode):
    app = Flask(__name__)
    layout = load_json(LAYOUT_PATH)
    bench = None
    startup_error = None
    if mode == "bench":
        try:
            bench = BenchHardware(load_json(BENCH_PATH))
            bench.start()
        except Exception as error:
            startup_error = str(error)

    @app.get("/")
    def index():
        return render_template_string(PAGE, mode=mode)

    @app.errorhandler(404)
    @app.errorhandler(502)
    @app.errorhandler(503)
    def api_error(error):
        if request.path.startswith("/api/"):
            return jsonify(error=error.description), error.code
        return error

    @app.get("/api/status")
    def status():
        thermal_indexes = {
            source.get("pair_name"): index
            for index, (source, _sensor) in enumerate(bench.thermals)
            if source.get("pair_name")
        } if bench else {}
        return jsonify({
            "mode": mode,
            "message": startup_error or ("Hardware preview ready" if bench else "Inspection artifact viewer ready"),
            "calibrated": bool(layout.get("calibrated")),
            "visible": list(bench.cameras) if bench else [],
            "thermal": [source for source, _sensor in bench.thermals] if bench else [],
            "radar": bench.radar_status() if bench else {"enabled": False, "detected": False},
            "pairs": [
                {
                    "name": pair["name"],
                    "visible_camera_index": pair["visible_camera_index"],
                    "bench_thermal_index": thermal_indexes.get(pair["name"]),
                }
                for pair in layout.get("pairs", [])
            ],
        })

    @app.get("/api/preview/visible/<int:index>.jpg")
    def visible_preview(index):
        if bench is None:
            abort(503, startup_error or "Bench hardware is unavailable")
        try:
            return preview_response(cv2.resize(bench.visible(index), (960, 540)))
        except KeyError:
            abort(404)

    @app.get("/api/preview/thermal/<int:index>.jpg")
    def thermal_preview_route(index):
        if bench is None:
            abort(503, startup_error or "Bench hardware is unavailable")
        try:
            source, frame = bench.thermal(index)
            return preview_response(bench.stamp_thermal(thermal_preview(frame, layout["thermal_range_c"]), source["name"]))
        except IndexError:
            abort(404)
        except Exception as error:
            abort(502, f"Thermal read failed: {error}")

    @app.post("/api/snapshot")
    def snapshot():
        if bench is None:
            return jsonify(error=startup_error or "Bench hardware is unavailable"), 503
        event_id = datetime.now().strftime("bench_%Y%m%d_%H%M%S_%f")
        try:
            for index in bench.cameras:
                cv2.imwrite(str(CAPTURE_DIR / f"{event_id}_camera{index}.jpg"), bench.visible(index))
            for index in range(len(bench.thermals)):
                source, frame = bench.thermal(index)
                np.save(CAPTURE_DIR / f"{event_id}_{source['name']}.npy", frame)
                cv2.imwrite(str(CAPTURE_DIR / f"{event_id}_{source['name']}.png"), thermal_preview(frame, layout["thermal_range_c"]))
        except Exception as error:
            return jsonify(error=f"Bench snapshot failed: {error}"), 502
        return jsonify(event_id=event_id)

    @app.post("/api/calibration/homography")
    def save_homography():
        body = request.get_json(silent=True) or {}
        pair = next((pair for pair in layout.get("pairs", []) if pair["name"] == body.get("pair")), None)
        source_points = np.asarray(body.get("source_points"), dtype=np.float32)
        canvas_points = np.asarray(body.get("canvas_points"), dtype=np.float32)
        if pair is None or body.get("source") not in {"visible", "thermal"}:
            return jsonify(error="Select a known pair and visible or thermal source"), 400
        if source_points.shape != canvas_points.shape or source_points.ndim != 2 or source_points.shape[0] < 4 or source_points.shape[1] != 2:
            return jsonify(error="Provide four or more [x, y] source and canvas point pairs"), 400
        homography, mask = cv2.findHomography(source_points, canvas_points, cv2.RANSAC)
        if homography is None or int(mask.sum()) < 4:
            return jsonify(error="Points do not produce a valid homography"), 400
        projected = cv2.perspectiveTransform(source_points[None, :, :], homography)[0]
        rms = float(np.sqrt(np.mean(np.sum((projected - canvas_points) ** 2, axis=1))))
        pair[f"{body['source']}_to_canvas"] = homography.tolist()
        layout["calibrated"] = False
        with LAYOUT_PATH.open("w", encoding="utf-8") as destination:
            json.dump(layout, destination, indent=2)
        return jsonify(rms=rms)

    @app.post("/api/calibration/relative")
    def save_relative_transform():
        body = request.get_json(silent=True) or {}
        reference = next((pair for pair in layout.get("pairs", []) if pair["name"] == body.get("reference_pair")), None)
        target = next((pair for pair in layout.get("pairs", []) if pair["name"] == body.get("target_pair")), None)
        reference_source = body.get("reference_source")
        target_source = body.get("target_source")
        stage_transform = np.asarray(body.get("stage_transform"), dtype=np.float64)
        stage_size = body.get("stage_size")
        if reference is None or target is None or reference_source not in {"visible", "thermal"} or target_source not in {"visible", "thermal"}:
            return jsonify(error="Select known reference and target pair sources"), 400
        if stage_transform.shape != (3, 3) or not np.isfinite(stage_transform).all() or np.isclose(stage_transform[2, 2], 0):
            return jsonify(error="Stage transform must be a finite 3x3 matrix"), 400
        if not isinstance(stage_size, list) or len(stage_size) != 2 or any(not isinstance(value, (int, float)) or value <= 0 for value in stage_size):
            return jsonify(error="Stage size must contain positive width and height"), 400
        reference_transform = reference.get(to_canvas_key(reference_source))
        if reference_transform is None:
            return jsonify(error=f"Anchor {reference['name']} {reference_source} to the shared canvas before relative registration"), 400
        reference_transform = np.asarray(reference_transform, dtype=np.float64)
        if reference_transform.shape != (3, 3) or not np.isfinite(reference_transform).all():
            return jsonify(error="Reference canvas transform is invalid"), 400

        # Browser transform is target-stage -> reference-stage. Convert it to target pixels -> canvas pixels.
        target_transform = reference_transform @ np.linalg.inv(source_to_stage(reference_source, stage_size)) @ stage_transform @ source_to_stage(target_source, stage_size)
        target[to_canvas_key(target_source)] = (target_transform / target_transform[2, 2]).tolist()
        layout["calibrated"] = False
        with LAYOUT_PATH.open("w", encoding="utf-8") as destination:
            json.dump(layout, destination, indent=2)
        return jsonify(target_key=to_canvas_key(target_source), transform=target[to_canvas_key(target_source)])

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
