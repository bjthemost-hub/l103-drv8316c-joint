# -*- coding: utf-8 -*-
"""给 CH32L103_DRV8316C_FOC_V2 补一套 MounRiver Studio 2 工程文件 (7 份)。
模板 = 同目录的旧工程 CH32L103_DRV8316C_FOC_CAN (同一颗 CH32L103K8U6 / 同一台 MRS 安装,
       2026-10-13 在 MRS + WCH-Link 上真编过、真烧过)。
只改: 工程名 + 绝对路径 + sourceEntries 的排除目录 (work/build/docs/scripts),
Makefile 那套原样不动, 两套产物各在各的目录 (build\\ 与 obj\\)。
"""
import io, json, os, re, sys, xml.etree.ElementTree as ET

OLD, NEW = "L103_DRV8316C_FOC", "L103_DRV8316C_FOC_V2"
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "CH32L103_DRV8316C_FOC_CAN")  # 旧模板工程 (可选)
DST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # firmware 根
DSTL = DST.lower()

def rd(base, p):
    return io.open(os.path.join(base, p), encoding="utf-8", newline="").read()

def wr(p, t):
    fp = os.path.join(DST, p)
    d = os.path.dirname(fp)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    io.open(fp, "w", encoding="utf-8", newline="").write(t)
    print("  + %-40s %7d chars" % (p, len(t)))

# 1) .project  (只换工程名; filteredResources 里那些 .bak 过滤项对 V2 没用, 收掉)
t = rd(SRC, ".project")
assert t.count("<name>%s</name>" % OLD) == 1
t = t.replace("<name>%s</name>" % OLD, "<name>%s</name>" % NEW)
t = re.sub(r"[ \t]*<filter>\s*<name/>\s*<type>6</type>.*?</filter>\n?", "", t, flags=re.S)
t = re.sub(r"[ \t]*<filter>\s*<name>User</name>.*?</filter>\n?", "", t, flags=re.S)
assert "<name>%s</name>" % NEW in t and OLD not in t.replace(NEW, "")
wr(".project", t)

# 2) .cproject  (排除目录: 别让 MRS 去编 work\\ 里的旧代码和 build\\ 里的中间产物)
t = rd(SRC, ".cproject")
old_ex = 'excluding="User/main_i2conly.c|Startup/startup_ch32v20x_D8W.S|Startup/startup_ch32v20x_D8.S"'
new_ex = 'excluding="User/main_i2conly.c|Startup/startup_ch32v20x_D8W.S|Startup/startup_ch32v20x_D8.S|work|build|docs|scripts|obj"'
assert t.count('${workspace_loc:/%s/obj' % OLD) == 1
t = t.replace('${workspace_loc:/%s/obj' % OLD, '${workspace_loc:/%s/obj' % NEW)
assert t.count(old_ex) == 1, t.count(old_ex)
t = t.replace(old_ex, new_ex)
assert OLD not in t.replace(NEW, "")
assert "${ProjName}/User" in t and "${ProjName}/SRC/Ld/Link.ld" in t
wr(".cproject", t)

# 3) .launch (CRLF 保持)
t = rd(SRC, ".launch")
t = t.replace('value="obj\\%s.elf"' % OLD, 'value="obj\\%s.elf"' % NEW)
t = t.replace('value="%s" />' % OLD, 'value="%s" />' % NEW)
assert OLD not in t.replace(NEW, "") and NEW in t
wr(".launch", t)

# 4) .template (CRLF 保持)
t = rd(SRC, ".template")
t = t.replace("Target Path=obj/%s.hex" % OLD, "Target Path=obj/%s.hex" % NEW)
assert OLD not in t.replace(NEW, "") and "MCU=CH32L103K8U" in t and "PeripheralVersion=2.0" in t
wr(".template", t)

# 5) <name>.wvproj
d = json.loads(rd(SRC, "%s.wvproj" % OLD))
d["basic"]["projectName"] = NEW
d["flashConfig"]["target_path"] = "obj/%s.hex" % NEW
d["debugConfigurations"]["reserve"]["PROGRAM_NAME"] = "obj/%s.elf" % NEW
d["debugConfigurations"]["reserve"]["PROJECT_ATTR"] = NEW
d["removedResources"] = [{"parentLogicPath": "", "type": "file", "name": "*.wvproj"}]
cfg = d["buildConfig"]["configurations"][0]
inc = cfg["ccompiler"]["includes"]["include_paths"]
assert inc == ["${project}/SRC/Debug", "${project}/SRC/Core", "${project}/User",
               "${project}/SRC/Peripheral/inc"], inc
assert cfg["clinker"]["general"]["scriptFiles"] == ["${project}/SRC/Ld/Link.ld"]
assert cfg["riscvTargetProcessor"]["architecture"] == "rv32i"
assert cfg["optimization"]["level"] == "size"
t = json.dumps(d, ensure_ascii=False, indent="\t")
assert OLD not in t.replace(NEW, "")
wr("%s.wvproj" % NEW, t)

# 6) .mrs/<name>.mrs-workspace
ws = json.loads(rd(SRC, os.path.join(".mrs", "%s.mrs-workspace" % OLD)))
ws["folders"] = [{"path": "."}, {"name": NEW, "path": DST}]
t = json.dumps(ws, ensure_ascii=False, indent="\t")
assert OLD not in t.replace(NEW, "")
wr(os.path.join(".mrs", "%s.mrs-workspace" % NEW), t)

# 7) .mrs/launch.json
lj = json.loads(rd(SRC, os.path.join(".mrs", "launch.json")))
c = lj["configurations"][0]
c["name"] = NEW
c["cwd"] = DSTL
c["startup"]["loadedFiles"]["executableFile"] = DSTL + "\\obj\\%s.elf" % NEW
c["startup"]["loadedFiles"]["symbolFile"]     = DSTL + "\\obj\\%s.elf" % NEW
t = json.dumps(lj, ensure_ascii=False, indent="\t")
assert OLD not in t.replace(NEW, "")
wr(os.path.join(".mrs", "launch.json"), t)
print("GENERATE OK")
