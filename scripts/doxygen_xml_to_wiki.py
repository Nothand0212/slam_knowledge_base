#!/usr/bin/env python3
"""
doxygen_xml_to_wiki.py - Convert Doxygen XML to GTSAM API wiki pages.

Reads Doxygen XML output and generates structured Markdown wiki pages for
key GTSAM C++ classes, suitable for the llm-wiki knowledge base.

Usage:
    python3 doxygen_xml_to_wiki.py <xml_dir> <wiki_output_dir>
"""

import xml.etree.ElementTree as ET
import os
import re
import sys
from pathlib import Path


# ============================================================
# Configuration
# ============================================================

# Classes to generate wiki pages for, grouped by category.
# Maps: wiki_category -> [(short_class_name, xml_refid), ...]
TARGET_CLASSES = {
    "Geometry": [
        ("Pose2", "classgtsam_1_1_pose2"),
        ("Pose3", "classgtsam_1_1_pose3"),
        ("Rot2", "classgtsam_1_1_rot2"),
        ("Rot3", "classgtsam_1_1_rot3"),
        ("Cal3_S2", "classgtsam_1_1_cal3___s2"),
        ("Cal3_S2Stereo", "classgtsam_1_1_cal3___s2_stereo"),
        ("StereoCamera", "classgtsam_1_1_stereo_camera"),
        ("PinholeCamera", "classgtsam_1_1_pinhole_camera"),
    ],
    "FactorGraph": [
        ("NonlinearFactorGraph", "classgtsam_1_1_nonlinear_factor_graph"),
        ("Values", "class_values"),
        ("Symbol", "classgtsam_1_1_symbol"),
        ("GaussianFactorGraph", "classgtsam_1_1_gaussian_factor_graph"),
    ],
    "Optimization": [
        ("LevenbergMarquardtOptimizer", "classgtsam_1_1_levenberg_marquardt_optimizer"),
        ("GaussNewtonOptimizer", "classgtsam_1_1_gauss_newton_optimizer"),
        ("DoglegOptimizer", "classgtsam_1_1_dogleg_optimizer"),
        ("LevenbergMarquardtParams", "classgtsam_1_1_levenberg_marquardt_params"),
        ("GaussNewtonParams", "classgtsam_1_1_gauss_newton_params"),
        ("IterativeOptimizationParameters", "classgtsam_1_1_iterative_optimization_parameters"),
    ],
    "ISAM2": [
        ("ISAM2", "classgtsam_1_1_i_s_a_m2"),
        ("ISAM2Params", "structgtsam_1_1_i_s_a_m2_params"),
        ("ISAM2Result", "structgtsam_1_1_i_s_a_m2_result"),
        ("IncrementalFixedLagSmoother", "classgtsam_1_1_incremental_fixed_lag_smoother"),
    ],
    "SLAM_Factors": [
        ("PriorFactor", "classgtsam_1_1_prior_factor"),
        ("BetweenFactor", "classgtsam_1_1_between_factor"),
        ("GenericProjectionFactor", "classgtsam_1_1_generic_projection_factor"),
        ("SmartProjectionFactor", "classgtsam_1_1_smart_projection_factor"),
        ("SmartProjectionPoseFactor", "classgtsam_1_1_smart_projection_pose_factor"),
        ("GenericStereoFactor", "classgtsam_1_1_generic_stereo_factor"),
        ("NonlinearEquality", "classgtsam_1_1_nonlinear_equality"),
    ],
    "Navigation": [
        ("NavState", "classgtsam_1_1_nav_state"),
        ("PreintegrationParams", "structgtsam_1_1_preintegration_params"),
        ("PreintegratedImuMeasurements", "classgtsam_1_1_preintegrated_imu_measurements_t"),
        ("ImuFactorT", "classgtsam_1_1_imu_factor_t"),
        ("CombinedImuFactorT", "classgtsam_1_1_combined_imu_factor_t"),
        ("ImuFactor2T", "classgtsam_1_1_imu_factor2_t"),
        ("GPSFactor", "classgtsam_1_1_g_p_s_factor"),
        ("GPSFactor2", "classgtsam_1_1_g_p_s_factor2"),
        ("ConstantBias", "classgtsam_1_1imu_bias_1_1_constant_bias"),
        ("PreintegratedAhrsMeasurements", "classgtsam_1_1_preintegrated_ahrs_measurements"),
    ],
    "Inference": [
        ("BayesNet", "classgtsam_1_1_bayes_net"),
        ("BayesTree", "classgtsam_1_1_bayes_tree"),
        ("HessianFactor", "classgtsam_1_1_hessian_factor"),
        ("JacobianFactor", "classgtsam_1_1_jacobian_factor"),
        ("Marginals", "classgtsam_1_1_marginals"),
        ("Ordering", "classgtsam_1_1_ordering"),
    ],
}

# Today's date for frontmatter
TODAY = "2026-05-29"

# Source info
GTSAM_SOURCE = "raw/codes/gtsam"
GTSAM_BRANCH = "develop"
GTSAM_REMOTE = "https://github.com/borglab/gtsam.git"
GTSAM_DOXYGEN_URL = "https://gtsam.org/doxygen/"

# gtsam_points source info
GTSAM_POINTS_SOURCE = "raw/codes/gtsam_points"
GTSAM_POINTS_VERSION = "v1.2.1"
GTSAM_POINTS_REMOTE = "https://github.com/koide3/gtsam_points.git"
GTSAM_POINTS_DOXYGEN_URL = "https://koide3.github.io/gtsam_points/doc_cpp/index.html"

# gtsam_points target classes
TARGET_CLASSES_GTSAM_POINTS = {
    "Scan Matching Factors": [
        ("IntegratedMatchingCostFactor", "classgtsam__points_1_1IntegratedMatchingCostFactor"),
        ("IntegratedICPFactor_", "classgtsam__points_1_1IntegratedICPFactor__"),
        ("IntegratedPointToPlaneICPFactor_", "classgtsam__points_1_1IntegratedPointToPlaneICPFactor__"),
        ("IntegratedGICPFactor_", "classgtsam__points_1_1IntegratedGICPFactor__"),
        ("IntegratedVGICPFactor_", "classgtsam__points_1_1IntegratedVGICPFactor__"),
        ("IntegratedVGICPFactorGPU", "classgtsam__points_1_1IntegratedVGICPFactorGPU"),
        ("IntegratedLOAMFactor_", "classgtsam__points_1_1IntegratedLOAMFactor__"),
        ("IntegratedPointToPlaneFactor_", "classgtsam__points_1_1IntegratedPointToPlaneFactor__"),
    ],
    "Colored & Continuous Factors": [
        ("IntegratedColorConsistencyFactor_", "classgtsam__points_1_1IntegratedColorConsistencyFactor__"),
        ("IntegratedColoredGICPFactor_", "classgtsam__points_1_1IntegratedColoredGICPFactor__"),
        ("IntegratedCT_ICPFactor_", "classgtsam__points_1_1IntegratedCT__ICPFactor__"),
        ("IntegratedCT_GICPFactor_", "classgtsam__points_1_1IntegratedCT__GICPFactor__"),
    ],
    "Bundle Adjustment": [
        ("BundleAdjustmentFactorBase", "classgtsam__points_1_1BundleAdjustmentFactorBase"),
        ("EVMBundleAdjustmentFactorBase", "classgtsam__points_1_1EVMBundleAdjustmentFactorBase"),
        ("EdgeEVMFactor", "classgtsam__points_1_1EdgeEVMFactor"),
        ("PlaneEVMFactor", "classgtsam__points_1_1PlaneEVMFactor"),
        ("LsqBundleAdjustmentFactor", "classgtsam__points_1_1LsqBundleAdjustmentFactor"),
        ("LinearDampingFactor", "classgtsam__points_1_1LinearDampingFactor"),
    ],
    "Nearest Neighbor": [
        ("KdTree", "structgtsam__points_1_1KdTree"),
        ("KdTreeX", "structgtsam__points_1_1KdTreeX"),
        ("IncrementalVoxelMap", "structgtsam__points_1_1IncrementalVoxelMap"),
        ("IncrementalCovarianceVoxelMap", "structgtsam__points_1_1IncrementalCovarianceVoxelMap"),
        ("FastOccupancyGrid", "classgtsam__points_1_1FastOccupancyGrid"),
        ("GaussianVoxelMap", "classgtsam__points_1_1GaussianVoxelMap"),
        ("GaussianVoxelMapCPU", "classgtsam__points_1_1GaussianVoxelMapCPU"),
    ],
    "Point Cloud & Trajectory": [
        ("PointCloud", "structgtsam__points_1_1PointCloud"),
        ("PointCloudCPU", "structgtsam__points_1_1PointCloudCPU"),
        ("ContinuousTrajectory", "classgtsam__points_1_1ContinuousTrajectory"),
        ("RegistrationResult", "structgtsam__points_1_1RegistrationResult"),
        ("Pose3InterpolationFactor", "classgtsam__points_1_1Pose3InterpolationFactor"),
    ],
    "CUDA": [
        ("NonlinearFactorSetGPU", "classgtsam__points_1_1NonlinearFactorSetGPU"),
        ("NonlinearFactorGPU", "classgtsam__points_1_1NonlinearFactorGPU"),
        ("CUDABuffer", "classgtsam__points_1_1CUDABuffer"),
        ("CUDAGraphExec", "classgtsam__points_1_1CUDAGraphExec"),
    ],
}


# ============================================================
# XML Parsing
# ============================================================

def parse_class_xml(xml_path):
    """Parse a class/struct Doxygen XML file and return structured data."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    compound = root.find('compounddef')
    if compound is None:
        return None

    data = {
        'kind': compound.get('kind'),
        'name': compound.findtext('compoundname', ''),
        'short_name': compound.findtext('compoundname', '').split('::')[-1],
        'header': '',
        'brief': '',
        'detail': '',
        'base_classes': [],
        'derived_classes': [],
        'members': [],  # list of dicts
        'typedefs': [],
        'enums': [],
        'inner_classes': [],
    }

    # Strip template parameters for short name
    sn = data['short_name']
    if '<' in sn:
        sn = sn.split('<')[0]
        data['short_name'] = sn

    # Header file
    for inc in compound.findall('includes'):
        if inc.get('local') == 'no':
            data['header'] = inc.text or ''

    # Brief description
    brief_el = compound.find('briefdescription')
    if brief_el is not None:
        paras = []
        for p in brief_el.findall('.//para'):
            text = get_text_content(p)
            if text.strip():
                paras.append(text.strip())
        data['brief'] = ' '.join(paras)

    # Detailed description
    detail_el = compound.find('detaileddescription')
    if detail_el is not None:
        paras = []
        for p in detail_el.findall('.//para'):
            text = get_text_content(p)
            if text.strip():
                paras.append(text.strip())
        data['detail'] = ' '.join(paras)

    # Base classes
    for bc in compound.findall('basecompoundref'):
        name = bc.text or ''
        if name:
            data['base_classes'].append(name)

    # Members - grouped by section
    for section in compound.findall('sectiondef'):
        section_kind = section.get('kind', 'public-func')
        for member in section.findall('memberdef'):
            member_data = parse_member(member, section_kind)
            if member_data:
                data['members'].append(member_data)

    return data


def get_text_content(element):
    """Get flattened text content, handling refs and other inline elements."""
    if element is None:
        return ''
    parts = []
    if element.text:
        parts.append(element.text)
    for child in element:
        if child.tag == 'ref':
            parts.append(child.text or '')
        elif child.tag == 'para':
            parts.append(get_text_content(child))
        elif child.tag == 'computeroutput':
            parts.append(f"`{get_text_content(child)}`")
        elif child.tag == 'emphasis':
            parts.append(f"*{get_text_content(child)}*")
        elif child.tag == 'bold':
            parts.append(f"**{get_text_content(child)}**")
        else:
            parts.append(get_text_content(child))
        if child.tail:
            parts.append(child.tail)
    return ''.join(parts)


def parse_member(member_el, section_kind):
    """Parse a memberdef element into a structured dict."""
    kind = member_el.get('kind', '')
    prot = member_el.get('prot', 'public')
    static = member_el.get('static', 'no') == 'yes'
    const = member_el.get('const', 'no') == 'yes'

    name = member_el.findtext('name', '')
    if not name:
        return None

    member = {
        'kind': kind,
        'prot': prot,
        'static': static,
        'const': const,
        'name': name,
        'section': section_kind,
        'brief': '',
        'signature': '',
        'params': [],
        'return_type': '',
        'definition': '',
    }

    # Brief description
    brief_el = member_el.find('briefdescription')
    if brief_el is not None:
        paras = brief_el.findall('.//para')
        texts = [get_text_content(p).strip() for p in paras if get_text_content(p).strip()]
        member['brief'] = ' '.join(texts)

    # Return type
    type_el = member_el.find('type')
    if type_el is not None:
        member['return_type'] = simplify_type(get_text_content(type_el).strip())

    # Parameters
    for param in member_el.findall('param'):
        ptype_el = param.find('type')
        ptype = get_text_content(ptype_el).strip() if ptype_el is not None else ''
        pname = param.findtext('declname', '')
        defval = param.findtext('defval', '')
        member['params'].append({
            'type': simplify_type(ptype),
            'name': pname,
            'default': defval,
        })

    # Full definition
    defn = member_el.findtext('definition', '')
    args = member_el.findtext('argsstring', '')
    if defn:
        member['definition'] = simplify_type(defn.strip()) + (args or '')

    # Build a clean signature
    member['signature'] = build_signature(member)

    return member


def simplify_type(type_str):
    """Simplify noisy C++ type strings for readability."""
    if not type_str:
        return type_str
    # Remove excessive whitespace
    type_str = re.sub(r'\s+', ' ', type_str).strip()
    # Shorten common patterns
    type_str = type_str.replace('gtsam::', '')
    type_str = type_str.replace('noiseModel::', '')
    type_str = type_str.replace('mEstimator::', '')
    return type_str


def build_signature(member):
    """Build a readable function signature string."""
    static = 'static ' if member['static'] else ''
    const = ' const' if member['const'] else ''

    if member['kind'] == 'function':
        ret = f"{member['return_type']} " if member['return_type'] and member['return_type'] != 'void' else ''
        params = []
        for p in member['params']:
            if p['default']:
                params.append(f"{p['type']} {p['name']} = {p['default']}")
            else:
                params.append(f"{p['type']} {p['name']}")
        param_str = ', '.join(params) if params else ''
        return f"{static}{ret}{member['name']}({param_str}){const}"
    elif member['kind'] == 'typedef':
        return f"using {member['name']} = {member['return_type']}"
    elif member['kind'] == 'variable':
        return f"{member['return_type']} {member['name']}"
    elif member['kind'] == 'enum':
        return f"enum {member['name']}"
    return f"{static}{member['return_type']} {member['name']}"


# ============================================================
# Wiki Page Generation
# ============================================================

def generate_wiki_page(data, category, project="gtsam"):
    """Generate a markdown wiki page from parsed class data."""
    name = data['short_name']
    kind = data['kind']
    kind_cn = '类' if kind == 'class' else '结构体'
    header_file = data['header'] or f"gtsam/{category.lower()}/{name}.h"

    if project == "gtsam_points":
        source_info = f"`{GTSAM_POINTS_SOURCE}` ({GTSAM_POINTS_VERSION})"
        remote_info = GTSAM_POINTS_REMOTE
        doc_url = GTSAM_POINTS_DOXYGEN_URL
        index_page = "GTSAM C++ API 参考索引"
        api_family = "方法-gtsam_points因子封装模式"
    else:
        source_info = f"`{GTSAM_SOURCE}` ({GTSAM_BRANCH})"
        remote_info = GTSAM_REMOTE
        doc_url = GTSAM_DOXYGEN_URL
        index_page = "GTSAM C++ API 参考索引"
        api_family = "方法-GTSAM-API族"

    lines = []
    # Frontmatter
    lines.append('---')
    lines.append(f'type: entity')
    tag_proj = "gtsam_points" if project == "gtsam_points" else "GTSAM"
    lines.append(f'tags: [{tag_proj}, C++ API, {category}, {name}]')
    lines.append(f'created: {TODAY}')
    lines.append(f'updated: {TODAY}')
    lines.append('sources:')
    lines.append(f'  - {doc_url}')
    lines.append(f'  - {source_info.split("(")[0].strip().strip("`")}')
    lines.append('---')
    lines.append('')
    proj_name_disp = "gtsam_points" if project == "gtsam_points" else "GTSAM"
    lines.append(f'# {proj_name_disp}::{name}')
    lines.append('')
    lines.append(f'> **{kind_cn}** | 头文件: `{header_file}` | [在线文档]({doc_url})')
    lines.append('')

    # Brief description
    if data['brief']:
        lines.append(data['brief'])
        lines.append('')

    # Inheritance
    if data['base_classes']:
        lines.append('## 继承关系')
        lines.append('')
        for bc in data['base_classes']:
            short = bc.split('::')[-1]
            lines.append(f'- 继承自 `{bc}`')
        lines.append('')

    # Public constructors
    constructors = [m for m in data['members'] if m['kind'] == 'function' and m['prot'] == 'public' and m['name'] == name]
    if constructors:
        lines.append('## 构造函数')
        lines.append('')
        for c in constructors:
            lines.append(f'```cpp')
            lines.append(c['signature'])
            lines.append(f'```')
            if c['brief']:
                lines.append(f'{c["brief"]}')
            lines.append('')

    # Public methods (non-constructor)
    pub_methods = [m for m in data['members'] if m['kind'] == 'function' and m['prot'] == 'public' and m['name'] != name]
    # Remove destructors
    pub_methods = [m for m in pub_methods if not m['name'].startswith('~')]
    # Remove operator overloads (keep only important ones)
    # Group by section
    sections = {}
    for m in pub_methods:
        sec = m.get('section', 'public-func')
        if sec not in sections:
            sections[sec] = []
        sections[sec].append(m)

    if pub_methods:
        lines.append('## 公开方法')
        lines.append('')

        for sec, methods in sections.items():
            if not methods:
                continue
            sec_labels = {
                'public-func': '### 方法',
                'public-static-func': '### 静态方法',
                'public-type': '### 类型别名',
                'user-defined': '### 方法',
            }
            label = sec_labels.get(sec, f'### {sec}')
            lines.append(label)
            lines.append('')

            for m in methods:
                brief = f' — {m["brief"]}' if m['brief'] else ''
                lines.append(f'```cpp')
                lines.append(m['signature'])
                lines.append(f'```')
                if m['brief']:
                    lines.append(f'{m["brief"]}')
                lines.append('')

    # Public typedefs
    typedefs = [m for m in data['members'] if m['kind'] == 'typedef' and m['prot'] == 'public']
    if typedefs:
        lines.append('## 类型别名')
        lines.append('')
        for t in typedefs:
            lines.append(f'```cpp')
            lines.append(t['signature'])
            lines.append(f'```')
        lines.append('')

    # Key member variables (public)
    pub_vars = [m for m in data['members'] if m['kind'] == 'variable' and m['prot'] == 'public']
    if pub_vars:
        lines.append('## 公开成员变量')
        lines.append('')
        for v in pub_vars:
            lines.append(f'```cpp')
            lines.append(v['signature'])
            lines.append(f'```')
        lines.append('')

    # Detailed description
    if data['detail']:
        lines.append('## 详细说明')
        lines.append('')
        lines.append(data['detail'])
        lines.append('')

    # Source info
    lines.append('## 源码位置')
    lines.append('')
    lines.append(f'- 远程: {remote_info}')
    lines.append(f'- 本地快照: {source_info}')
    lines.append('')

    # Agent 实现提示 (minimal)
    lines.append('## Agent 实现提示')
    lines.append('')
    lines.append('### 适用场景')
    lines.append('')
    lines.append(f'`{name}` 用于 GTSAM factor graph 优化流程中。')
    lines.append('')
    lines.append('### 输入输出契约')
    lines.append('')
    lines.append(f'参见上方 [公开方法](#公开方法) 的签名。')
    lines.append('')
    lines.append('### 实现注意事项')
    lines.append('')
    lines.append('- 所有 Lie group 类型使用右扰动（right perturbation）约定')
    lines.append('- 使用 `OptionalJacobian` 参数可选的链式求导')
    lines.append('')

    # Related pages
    lines.append('## 相关页面')
    lines.append('')
    lines.append(f'- [[{api_family}]]')
    lines.append(f'- [[{index_page}]]')
    lines.append('')

    return '\n'.join(lines)


def generate_index_page(categories, output_dir, project="gtsam"):
    """Generate an index page for all generated API pages."""
    if project == "gtsam_points":
        proj_name = "gtsam_points"
        proj_ver = GTSAM_POINTS_VERSION
        doc_url = GTSAM_POINTS_DOXYGEN_URL
        index_title = "gtsam_points C++ API 参考索引"
        index_desc = "gtsam_points 源码"
        api_family = "方法-gtsam_points因子封装模式"
        tag_prefix = "gtsam_points"
        src_prefix = "include/gtsam_points"
    else:
        proj_name = "GTSAM"
        proj_ver = GTSAM_BRANCH
        doc_url = GTSAM_DOXYGEN_URL
        index_title = "GTSAM C++ API 参考索引"
        index_desc = "GTSAM 源码"
        api_family = "方法-GTSAM-API族"
        tag_prefix = "GTSAM"
        src_prefix = "gtsam"

    lines = []
    lines.append('---')
    lines.append(f'type: entity')
    lines.append(f'tags: [{tag_prefix}, C++ API, index]')
    lines.append(f'created: {TODAY}')
    lines.append(f'updated: {TODAY}')
    lines.append('sources:')
    lines.append(f'  - {doc_url}')
    lines.append('---')
    lines.append('')
    lines.append(f'# {index_title}')
    lines.append('')
    lines.append(f'> 自动生成自 Doxygen XML | {proj_name} `{proj_ver}` | {TODAY}')
    lines.append('')
    lines.append(f'本文档是从 {index_desc} 通过 Doxygen 生成的 C++ API 参考。包含 {sum(len(v) for v in categories.values())} 个核心类的构造函数、方法签名和参数说明。')
    lines.append('')
    lines.append(f'在线 C++ 文档: [{doc_url}]({doc_url})')
    lines.append('')

    for cat, classes in categories.items():
        lines.append(f'## {cat}')
        lines.append('')
        lines.append('| 类 | 类型 | 头文件 |')
        lines.append('|----|------|--------|')
        for cls_name, refid in classes:
            safe_name = cls_name.replace('<', r'\<').replace('>', r'\>')
            lines.append(f'| [[C++ API - {cls_name}]] | class | {src_prefix}/{cls_name}.h |')
        lines.append('')

    lines.append('## 相关页面')
    lines.append('')
    lines.append(f'- [[{api_family}]]')
    lines.append('')

    return '\n'.join(lines)


# ============================================================
# Main
# ============================================================

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <doxygen_xml_dir> <wiki_output_dir> [--project gtsam|gtsam_points]")
        sys.exit(1)

    xml_dir = Path(sys.argv[1])
    wiki_dir = Path(sys.argv[2])

    # Parse optional --project flag
    project = "gtsam"
    for i, arg in enumerate(sys.argv):
        if arg == "--project" and i + 1 < len(sys.argv):
            project = sys.argv[i + 1]
            break

    if project == "gtsam_points":
        target_classes = TARGET_CLASSES_GTSAM_POINTS
    else:
        target_classes = TARGET_CLASSES
    wiki_dir.mkdir(parents=True, exist_ok=True)

    # Parse index.xml to get all class refids
    index_path = xml_dir / 'index.xml'
    if not index_path.exists():
        print(f"ERROR: {index_path} not found")
        sys.exit(1)

    # Build a lookup: full_class_name -> refid
    index_tree = ET.parse(index_path)
    refid_map = {}
    for compound in index_tree.findall('compound'):
        refid = compound.get('refid', '')
        name_el = compound.find('name')
        if name_el is not None and name_el.text:
            refid_map[name_el.text] = refid

    # Also build lookup by refid
    refid_to_file = {}
    for f in xml_dir.glob('*.xml'):
        if f.name == 'index.xml':
            continue
        refid_to_file[f.stem] = str(f)

    generated = {}

    for category, classes in target_classes.items():
        generated[category] = []
        for cls_name, refid in classes:
            # Find the XML file
            xml_file = refid_to_file.get(refid)
            if not xml_file:
                print(f"  SKIP {cls_name}: XML file not found for refid '{refid}'")
                continue

            # Parse
            data = parse_class_xml(xml_file)
            if not data:
                print(f"  SKIP {cls_name}: Failed to parse XML")
                continue

            # Generate wiki page
            page_content = generate_wiki_page(data, category, project=project)
            page_filename = f"C++ API - {cls_name}.md"
            page_path = wiki_dir / page_filename
            with open(page_path, 'w') as f:
                f.write(page_content)

            generated[category].append((cls_name, refid))
            method_count = sum(1 for m in data['members'] if m['kind'] == 'function' and m['prot'] == 'public')
            print(f"  OK   {cls_name} ({method_count} public methods) -> {page_filename}")

    # Generate index page
    if generated:
        index_content = generate_index_page(generated, wiki_dir, project=project)
        index_path_out = wiki_dir / f'{project} C++ API 参考索引.md'
        with open(index_path_out, 'w') as f:
            f.write(index_content)
        print(f"\nIndex page: {index_path_out}")

    # Summary
    total = sum(len(v) for v in generated.values())
    print(f"\nDone! Generated {total} wiki pages in {wiki_dir}")


if __name__ == '__main__':
    main()
