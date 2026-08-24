"""Browser bench, calibration, and inspection-artifact viewer for the scanner."""

import json
import os
import threading
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
:root{color-scheme:dark;--bg:#101413;--panel:#18201e;--line:#31403b;--ink:#e8eee8;--muted:#a6b3aa;--accent:#d6f27d;--warning:#ffca68}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#22322c,var(--bg) 42rem);color:var(--ink);font:15px/1.45 system-ui,sans-serif}main{max-width:1400px;margin:auto;padding:28px}h1{font-size:clamp(1.7rem,4vw,3rem);margin:0}h2{font-size:1rem;text-transform:uppercase;letter-spacing:.09em;color:var(--muted)}header{display:flex;justify-content:space-between;gap:16px;align-items:end;border-bottom:1px solid var(--line);padding-bottom:20px}.mode{color:var(--accent);font-weight:700;text-transform:uppercase;letter-spacing:.12em}.notice{margin:18px 0;padding:12px 14px;background:#25251a;border-left:3px solid var(--warning);color:#f9e7bb}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:16px}.panel{background:color-mix(in srgb,var(--panel),transparent 7%);border:1px solid var(--line);padding:16px;border-radius:8px}.feed{width:100%;aspect-ratio:16/9;object-fit:contain;background:#090c0b;border:1px solid #27332f}.thermal{image-rendering:pixelated}.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:18px 0}button{border:0;border-radius:4px;background:var(--accent);color:#152014;padding:11px 15px;font-weight:750;cursor:pointer}button:disabled{opacity:.45;cursor:not-allowed}.status{color:var(--muted)}.scans{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:16px}.scan{border:1px solid var(--line);padding:10px;border-radius:7px}.scan img{width:100%;background:#090c0b}.scan a{color:var(--accent);text-decoration:none}.calibration{display:none}.calibration.active{display:block}textarea{width:100%;min-height:150px;background:#0c100f;color:var(--ink);border:1px solid var(--line);padding:10px;font:12px ui-monospace,monospace}@media(max-width:600px){main{padding:16px}header{align-items:start;flex-direction:column}}
</style></head><body><main>
<header><div><div class="mode" id="mode"></div><h1>Undercarriage Scan Console</h1></div><div class="status" id="status">Connecting...</div></header>
<div class="notice" id="notice"></div>
<section id="bench"><div class="controls"><button id="snapshot">Save bench snapshot</button><button id="toggle">Show calibration editor</button></div><div class="grid" id="feeds"></div>
<div class="panel calibration" id="calibration"><h2>Homography editor</h2><p>Enter four or more matching source and canvas points. This writes only the selected homography; it never sets <code>calibrated</code> to true.</p><label>Pair <select id="pair"></select></label> <label>Source <select id="source"><option value="visible">Visible</option><option value="thermal">Thermal</option></select></label><textarea id="points" placeholder='{"source_points":[[x,y],[x,y],[x,y],[x,y]],"canvas_points":[[x,y],[x,y],[x,y],[x,y]]}'></textarea><button id="saveCalibration">Calculate and save transform</button><span class="status" id="calibrationStatus"></span></div></section>
<section><h2>Technician inspections</h2><div class="scans" id="scans"></div></section>
</main><script>
const mode={{ mode|tojson }};let info={};
async function api(path,options){const r=await fetch(path,options);const data=await r.json();if(!r.ok)throw Error(data.error||r.statusText);return data}
function feed(src,label,thermal=false){return `<article class="panel"><h2>${label}</h2><img class="feed ${thermal?'thermal':''}" src="${src}?t=${Date.now()}" onerror="this.alt='Waiting for source'" alt="${label}"></article>`}
async function status(){try{info=await api('/api/status');document.querySelector('#mode').textContent=info.mode;document.querySelector('#status').textContent=info.message;document.querySelector('#notice').textContent=info.calibrated?'Calibration is marked active. Review artifacts and coverage before unattended operation.':'Bench mode is safe for uncalibrated hardware. Deployed capture refuses to produce inspection images until calibration is complete.';if(mode==='bench'){document.querySelector('#feeds').innerHTML=info.visible.map(i=>feed('/api/preview/visible/'+i+'.jpg','Visible camera '+i)).join('')+info.thermal.map((s,i)=>feed('/api/preview/thermal/'+i+'.jpg',s.name+' thermal',true)).join('');document.querySelector('#pair').innerHTML=info.pairs.map(p=>`<option value="${p.name}">${p.name}</option>`).join('')}else document.querySelector('#bench').hidden=true}catch(e){document.querySelector('#status').textContent=e.message}}
async function scans(){try{const data=await api('/api/scans');document.querySelector('#scans').innerHTML=data.scans.length?data.scans.map(s=>`<article class="scan"><a href="/captures/${s.image}" target="_blank"><img src="/captures/${s.image}" loading="lazy"><strong>${s.event_id}</strong></a><div class="status">${s.stats}</div></article>`).join(''):'<p class="status">No deployed inspection mosaics yet.</p>'}catch(e){document.querySelector('#scans').textContent=e.message}}
document.querySelector('#snapshot').onclick=async()=>{try{const r=await api('/api/snapshot',{method:'POST'});document.querySelector('#status').textContent='Saved '+r.event_id}catch(e){alert(e.message)}};
document.querySelector('#toggle').onclick=()=>document.querySelector('#calibration').classList.toggle('active');
document.querySelector('#saveCalibration').onclick=async()=>{try{const body=JSON.parse(document.querySelector('#points').value);body.pair=document.querySelector('#pair').value;body.source=document.querySelector('#source').value;const r=await api('/api/calibration/homography',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});document.querySelector('#calibrationStatus').textContent='Saved. RMS reprojection error: '+r.rms.toFixed(2)+' px'}catch(e){document.querySelector('#calibrationStatus').textContent=e.message}};
status();scans();setInterval(scans,10000);setInterval(()=>{if(mode==='bench')document.querySelectorAll('.feed').forEach(i=>i.src=i.src.split('?')[0]+'?t='+Date.now())},1000);
</script></body></html>"""


class BenchHardware:
    def __init__(self, config):
        self.lock = threading.Lock()
        self.cameras = {}
        self.thermals = []
        self.config = config

    def start(self):
        import adafruit_mlx90640
        import board
        import busio
        from adafruit_tca9548a import TCA9548A
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
        except Exception:
            self.close()
            raise

    def close(self):
        for camera in self.cameras.values():
            camera.close()
        self.cameras.clear()

    def visible(self, index):
        with self.lock:
            if index not in self.cameras:
                raise KeyError(index)
            return cv2.cvtColor(self.cameras[index].capture_array("main"), cv2.COLOR_RGB2BGR)

    def thermal(self, source_index):
        with self.lock:
            source, sensor = self.thermals[source_index]
            frame = np.zeros(24 * 32, dtype=np.float32)
            sensor.getFrame(frame)
            return source, frame.reshape(24, 32)


def load_json(path):
    with path.open(encoding="utf-8") as source:
        return json.load(source)


def jpeg(image, quality=85):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("Could not encode preview")
    return encoded.tobytes()


def thermal_preview(frame, thermal_range):
    low, high = thermal_range
    normalized = np.uint8(np.clip((frame - low) / (high - low), 0, 1) * 255)
    image = cv2.applyColorMap(normalized, cv2.COLORMAP_INFERNO)
    image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_NEAREST)
    cv2.putText(image, f"{np.nanmin(frame):.1f} to {np.nanmax(frame):.1f} C", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return image


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
        return jsonify({
            "mode": mode,
            "message": startup_error or ("Hardware preview ready" if bench else "Inspection artifact viewer ready"),
            "calibrated": bool(layout.get("calibrated")),
            "visible": list(bench.cameras) if bench else [],
            "thermal": [source for source, _sensor in bench.thermals] if bench else [],
            "pairs": [{"name": pair["name"]} for pair in layout.get("pairs", [])],
        })

    @app.get("/api/preview/visible/<int:index>.jpg")
    def visible_preview(index):
        if bench is None:
            abort(503, startup_error or "Bench hardware is unavailable")
        try:
            return Response(jpeg(cv2.resize(bench.visible(index), (960, 540))), mimetype="image/jpeg")
        except KeyError:
            abort(404)

    @app.get("/api/preview/thermal/<int:index>.jpg")
    def thermal_preview_route(index):
        if bench is None:
            abort(503, startup_error or "Bench hardware is unavailable")
        try:
            _source, frame = bench.thermal(index)
            return Response(jpeg(thermal_preview(frame, layout["thermal_range_c"])), mimetype="image/jpeg")
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
