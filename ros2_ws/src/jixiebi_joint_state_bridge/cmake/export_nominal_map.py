"""构建时把原项目的线性模型导出给 C++；反馈节点运行时只需要 ROS。"""

from pathlib import Path
import sys

from arm_project.nominal_state_bridge import nominal_urdf_degrees


def main(output):
    # 在两个协议输入上求现有线性函数的截距与斜率。这里调用模型，不产生实测标定点。
    reference_p = 1500
    reference = nominal_urdf_degrees([reference_p] * 4)
    offset = nominal_urdf_degrees([reference_p + 100] * 4)
    gains = [(b - a) / 100 for a, b in zip(reference, offset)]
    output.write_text(
        "#pragma once\n#include <array>\nnamespace jixiebi {\n"
        f"inline constexpr int kVendorReferenceP = {reference_p};\n"
        "inline constexpr std::array<double, 4> kVendorReferenceDeg{"
        + ", ".join(map(repr, reference))
        + "};\ninline constexpr std::array<double, 4> kVendorDegreesPerP{"
        + ", ".join(map(repr, gains))
        + "};\n}\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main(Path(sys.argv[1]))
