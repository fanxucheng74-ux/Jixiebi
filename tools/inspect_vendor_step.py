"""Read vendor STEP with optional OCP8, preserving assembly placements and hashes.

Run in the isolated CAD environment; never imported by the arm runtime.
All dimensions refer to this CAD revision, not a verified physical instance.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_Label
    from OCP.collections import Sequence_TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.TopLoc import TopLoc_Location
    from OCP.IFSelect import IFSelect_ReturnStatus

    with zipfile.ZipFile(args.archive) as z:
        names = [n for n in z.namelist() if n.lower().endswith((".step", ".stp"))]
        if len(names) != 1:
            raise ValueError("select archive containing one STEP")
        data = z.read(names[0])
    args.output.mkdir(parents=True, exist_ok=False)
    step = args.output / "source.step"
    step.write_bytes(data)
    text = data.decode("gb18030")
    entities = {int(n): v for n, v in re.findall(r"#(\d+)\s*=\s*(.*?);", text, re.S)}

    def refs(value):
        return [int(v) for v in re.findall(r"#(\d+)", value)]

    def words(value):
        return re.findall(r"'((?:''|[^'])*)'", value)

    occurrence_names = {}
    for value in entities.values():
        if value.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
            child = refs(value)[1]
            formation = refs(entities[child])[0]
            product = refs(entities[formation])[0]
            occurrence_names[words(value)[0]] = words(entities[product])[0]
    if "SI_UNIT ( .MILLI., .METRE. )" not in text:
        raise ValueError("expected millimetre CAD units")
    doc = TDocStd_Document(TCollection_ExtendedString("vendor-audit"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(step)) != IFSelect_ReturnStatus.IFSelect_RetDone or not reader.Transfer(
        doc
    ):
        raise RuntimeError("STEP import failed")
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = Sequence_TDF_Label()
    st.GetFreeShapes(roots)

    def label_name(label):
        attr = TDataStd_Name()
        return (
            attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""
        )

    rows = []

    def walk(label, parent, path):
        ref = TDF_Label()
        use = ref if st.GetReferredShape_s(label, ref) else label
        shape = st.GetShape_s(label)
        location = parent.Multiplied(shape.Location())
        shape = shape.Located(location)
        occurrence = label_name(label)
        partpath = path + [occurrence]
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape, box, False, False)
        lo, hi = box.CornerMin(), box.CornerMax()
        bounds = [lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()]
        rows.append(
            {
                "occurrence_path": partpath,
                "occurrence": occurrence,
                "product_name": occurrence_names.get(occurrence, "CAD_ROOT"),
                "bounds_xyz_mm": bounds,
                "size_xyz_mm": [bounds[i + 3] - bounds[i] for i in range(3)],
                "centre_xyz_mm": [(bounds[i] + bounds[i + 3]) / 2 for i in range(3)],
            }
        )
        children = Sequence_TDF_Label()
        st.GetComponents_s(use, children, False)
        for i in range(1, children.Length() + 1):
            walk(children.Value(i), location, partpath)

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])

    def component(*path):
        found = [r for r in rows if r["occurrence_path"][1:] == list(path)]
        if len(found) != 1:
            raise ValueError("expected CAD occurrence not found: " + str(path))
        return found[0]

    board = component("NAUO23")
    base = component("NAUO7")["centre_xyz_mm"]
    shoulder = component("NAUO9", "NAUO67")["centre_xyz_mm"]
    elbow = component("NAUO11", "NAUO67")["centre_xyz_mm"]
    wrist = component("NAUO13", "NAUO67")["centre_xyz_mm"]
    if (
        board["product_name"] != "Jibot大底板"
        or max(abs(shoulder[j] - elbow[j]) for j in (1, 2)) > 1e-5
    ):
        raise ValueError(
            "unexpected CAD revision/axis orientation; do not reuse occurrence assumptions"
        )
    summary = {
        "vertical_axis_in_this_CAD": "positive_X",
        "table_x_mm": board["bounds_xyz_mm"][0],
        "board_planar_size_mm": board["size_xyz_mm"][1:],
        "CAD_shown_open_gripper_height_mm": rows[0]["size_xyz_mm"][0],
        "CAD_shoulder_axis_height_from_plate_bottom_mm": shoulder[0] - board["bounds_xyz_mm"][0],
        "CAD_upper_axis_distance_mm": abs(elbow[0] - shoulder[0]),
        "CAD_forearm_axis_distance_mm": abs(wrist[0] - elbow[0]),
        "CAD_wrist_to_current_open_max_tip_projection_mm": component("NAUO16")["bounds_xyz_mm"][3]
        - wrist[0],
        "axis_centre_to_plate_Y_minus_mm": base[1] - board["bounds_xyz_mm"][1],
        "axis_centre_to_plate_Y_plus_mm": board["bounds_xyz_mm"][4] - base[1],
        "axis_centre_to_plate_Z_minus_mm": base[2] - board["bounds_xyz_mm"][2],
        "axis_centre_to_plate_Z_plus_mm": board["bounds_xyz_mm"][5] - base[2],
        "physical_instance_verified": False,
        "closed_gripper_TCP_verified": False,
        "edge_direction_in_actual_arm_verified": False,
        "note": "Open-claw assembly dimensions and axis-centre projections; not closed-tool length, usable jaw opening or actual calibration.",
    }
    report = {
        "source_archive": str(args.archive),
        "archive_sha256": hashlib.sha256(args.archive.read_bytes()).hexdigest(),
        "step_sha256": hashlib.sha256(data).hexdigest(),
        "units": "mm",
        "reader": "OCP8.0.1.1.0 STEPCAFControl",
        "summary": summary,
        "components": rows,
    }
    (args.output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
