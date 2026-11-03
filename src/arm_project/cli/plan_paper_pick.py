"""Generate a non-executable known-position paper pickup and HTML preview."""

import argparse
import hashlib
import json
from pathlib import Path

from arm_project.paths import PROJECT_ROOT
from arm_project.pick_planning import plan_paper_pick


def write_preview(path, report):
    frames = []
    for segment in report["segments"]:
        samples = segment["samples"]
        stride = max(1, len(samples) // 180)
        chosen = samples[::stride]
        if chosen[-1] is not samples[-1]:
            chosen.append(samples[-1])
        frames.extend({"phase": segment["name"], **s} for s in chosen)
    payload = json.dumps(
        {
            "frames": frames,
            "table": report["table_z_in_model_mm"],
            "target": report["grasp_center_model_mm"],
            "radius": report["request"]["object_radius_mm"],
        },
        allow_nan=False,
    )
    html = """<!doctype html><meta charset="utf-8"><title>机械臂名义运动预览</title>
<style>body{font:16px system-ui;margin:24px;background:#f4f6fa;color:#18263c}main{max-width:1000px;margin:auto}svg{background:white;border:1px solid #ccd5e0;width:100%;height:590px}input{width:75%}pre{white-space:pre-wrap}.note{color:#a13c12}button{padding:8px 18px}</style>
<main><h2>纸团抓取：坐标与关节路径预览</h2><p class="note">仅厂家标称模型仿真，不连接机械臂；底座偏移、桌面Z和TCP等输入假设不能当成实测。侧视图显示前后/高度。</p>
<svg viewBox="0 0 950 620"><g id="drawing"></g></svg><p><button id="play">播放 / 暂停</button> <input id="slider" type="range" min="0" value="0"></p><pre id="readout"></pre></main>
<script>const D=PAYLOAD;const slider=document.querySelector('#slider');slider.max=D.frames.length-1;let playing=false;let index=0;const g=document.querySelector('#drawing');
const maxZ=Math.max(...D.frames.flatMap(f=>f.points_mm.map(p=>p[2]-D.table)),200)+30;const maxY=Math.max(...D.frames.flatMap(f=>f.points_mm.map(p=>Math.hypot(p[0],p[1]))),250)+50;const scale=Math.min(760/maxY,500/maxZ);const xy=p=>[70+Math.hypot(p[0],p[1])*scale,550-(p[2]-D.table)*scale];
function draw(){const f=D.frames[index],P=f.points_mm.map(xy),t=xy(D.target);let s='<line x1="40" y1="550" x2="920" y2="550" stroke="#667"/><text x="50" y="585">桌面；水平为距底座的径向距离，单位mm</text>';
s+=`<circle cx="${t[0]}" cy="${t[1]}" r="${D.radius*scale}" fill="#df8f8655" stroke="#aa493d"/>`;
for(let i=1;i<P.length;i++)s+=`<line x1="${P[i-1][0]}" y1="${P[i-1][1]}" x2="${P[i][0]}" y2="${P[i][1]}" stroke="#386ca6" stroke-width="9"/>`;
for(const p of P)s+=`<circle cx="${p[0]}" cy="${p[1]}" r="7" fill="#153859"/>`;g.innerHTML=s;
document.querySelector('#readout').textContent=`阶段：${f.phase}   本段：${f.time_s.toFixed(2)}s\nP000..003：[${f.pulses_p.join(', ')}]\n名义爪尖 XYZ(mm)：${[f.tip_model.x_mm,f.tip_model.y_mm,f.tip_model.z_mm-D.table].map(x=>x.toFixed(1)).join(', ')}\n俯仰角(标称)：${f.tip_model.pitch_deg.toFixed(1)}°\n纸团只作位置标记；夹爪宽度和夹持未仿真。`;slider.value=index;}
slider.oninput=()=>{index=Number(slider.value);draw()};document.querySelector('#play').onclick=()=>playing=!playing;setInterval(()=>{if(playing){index=(index+1)%D.frames.length;draw()}},50);draw();</script>""".replace(
        "PAYLOAD", payload
    )
    with path.open("x", encoding="utf-8") as f:
        f.write(html)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-file", type=Path, required=True)
    parser.add_argument(
        "--model", type=Path, default=PROJECT_ROOT / "configs/kinematics.vendor_nominal.json"
    )
    parser.add_argument(
        "--request", type=Path, default=PROJECT_ROOT / "configs/paper_pick.example.json"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        payloads = [p.read_bytes() for p in (args.state_file, args.model, args.request)]
        state, model, request = [json.loads(p.decode("utf-8-sig")) for p in payloads]
        if (
            state.get("source") != "HARDWARE"
            or state.get("error")
            or state.get("state", {}).get("status") != "OK"
        ):
            raise ValueError("need a complete recorded hardware read_state result")
        servos = state["state"]["servos"]
        if [s["servo_id"] for s in servos] != list(range(6)) or any(
            s["status"] != "OK" for s in servos
        ):
            raise ValueError("need six ordered successful state records")
        report = plan_paper_pick(model, request, [s["position_p"] for s in servos])
        report["recorded_state_at"] = state["state"].get("finished_at")
        report["input_sha256"] = {
            k: hashlib.sha256(v).hexdigest()
            for k, v in zip(("state", "model", "request"), payloads)
        }
        args.output.mkdir(parents=True, exist_ok=False)
        with (args.output / "proposal.json").open("x", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
        write_preview(args.output / "preview.html", report)
        print(
            json.dumps(
                {
                    k: report[k]
                    for k in (
                        "status",
                        "motion_authorized",
                        "distance_to_centre_mm",
                        "grasp_center_task_mm",
                        "grasp_pulses_p",
                        "approach_pulses_p",
                        "required_opening_mm",
                        "maximum_sampled_radius_mm",
                    )
                },
                ensure_ascii=False,
            )
        )
        print("OUTPUT:", args.output)
        return 0
    except (ValueError, KeyError, TypeError, OSError) as e:
        print(
            json.dumps(
                {"status": "NO_PROPOSAL", "error": str(e), "motion_authorized": False},
                ensure_ascii=False,
            )
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
