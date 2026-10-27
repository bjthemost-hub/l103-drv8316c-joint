# -*- coding: utf-8 -*-
"""校验 V2 的 7 份 MounRiver 工程文件: 能解析 / 无旧名残留 / 绝对路径真实存在 / 行尾与模板一致"""
import io, json, os, re, sys, xml.etree.ElementTree as ET

OLD, NEW = "L103_DRV8316C_FOC", "L103_DRV8316C_FOC_V2"
DST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # firmware 根
ok = True

def fail(msg):
    global ok
    ok = False
    print("  [FAIL] " + msg)

def raw(p):
    return io.open(os.path.join(DST, p), encoding="utf-8", newline="").read()

files = [".project", ".cproject", ".launch", ".template",
         NEW + ".wvproj", os.path.join(".mrs", NEW + ".mrs-workspace"), os.path.join(".mrs", "launch.json")]

print("1) 存在性 / 解析 / 旧名残留")
for p in files:
    fp = os.path.join(DST, p)
    if not os.path.isfile(fp):
        fail("缺文件 " + p); continue
    t = raw(p)
    if OLD in t.replace(NEW, ""):
        fail(p + " 里有旧工程名残留")
    if p.endswith(".json") or p.endswith("wvproj") or p.endswith(".mrs-workspace"):
        try: json.loads(t)
        except Exception as e: fail(p + " JSON 解析失败: " + str(e))
    elif p.endswith(".project") or p.endswith(".cproject"):
        try: ET.fromstring(t)
        except Exception as e: fail(p + " XML 解析失败: " + str(e))
    print("   %-46s %7d chars" % (p, len(t)))

print("2) 关键字段")
pj = ET.fromstring(raw(".project")); assert pj.findtext("name") == NEW
d = json.loads(raw(NEW + ".wvproj"))
if d["basic"]["projectName"] != NEW: fail("wvproj projectName")
if d["flashConfig"]["target_path"] != "obj/%s.hex" % NEW: fail("wvproj target_path")
r = d["debugConfigurations"]["reserve"]
if r["PROGRAM_NAME"] != "obj/%s.elf" % NEW or r["PROJECT_ATTR"] != NEW: fail("wvproj reserve")
c = d["buildConfig"]["configurations"][0]
for k, v in [("includes", c["ccompiler"]["includes"]["include_paths"]),
             ("script", c["clinker"]["general"]["scriptFiles"])]:
    print("   %-8s %s" % (k, v))
if c["clinker"]["libraries"]["libraries"] != ["m"]: fail("wvproj 链接库不是 [m]")
lr = raw(".launch")
if "obj\\%s.elf" % NEW not in lr: fail(".launch PROGRAM_NAME 没改到")
if "Target Path=obj/%s.hex" % NEW not in raw(".template"): fail(".template Target Path 没改到")

print("3) 绝对路径 / include / 链接脚本 真实存在")
lj = json.loads(raw(os.path.join(".mrs", "launch.json")))["configurations"][0]
oc = lj["openOCDCfg"]["executable"]; gd = lj["gdbCfg"]["executable"]
svd = lj["svdpath"]; ex = lj["startup"]["loadedFiles"]["executableFile"]
for label, path in [("openocd", oc), ("gdb", gd), ("svd", svd)]:
    if not os.path.isfile(path): fail("%s 不存在: %s" % (label, path))
    else: print("   %-8s OK  %s" % (label, path))
want_dir = DST.lower()
if lj["cwd"] != want_dir: fail("launch.json cwd=%s" % lj["cwd"])
if not ex.startswith(want_dir + "\\obj\\"): fail("launch.json executableFile=%s" % ex)
print("   obj elf  (编译后才有)  %s" % ex)
for inc in c["ccompiler"]["includes"]["include_paths"]:
    rel = inc.replace("${project}/", "").replace("/", os.sep)
    if not os.path.isdir(os.path.join(DST, rel)): fail("include 目录不存在: " + inc)
for sc in c["clinker"]["general"]["scriptFiles"]:
    rel = sc.replace("${project}/", "").replace("/", os.sep)
    if not os.path.isfile(os.path.join(DST, rel)): fail("链接脚本不存在: " + sc)
ws = json.loads(raw(os.path.join(".mrs", NEW + ".mrs-workspace")))
if ws["folders"][1]["path"] != DST: fail("mrs-workspace path")
if ws["folders"][1]["name"] != NEW: fail("mrs-workspace name")

print("4) 行尾 (LF: .project/.cproject/wvproj/.mrs   CRLF: .launch/.template)")
for p, want in [(".project", "LF"), (".cproject", "LF"), (NEW + ".wvproj", "LF"),
                (os.path.join(".mrs", NEW + ".mrs-workspace"), "LF"), (os.path.join(".mrs", "launch.json"), "LF"),
                (".launch", "CRLF"), (".template", "CRLF")]:
    b = io.open(os.path.join(DST, p), "rb").read()
    crlf = b.count(b"\r\n"); lf = b.count(b"\n")
    got = "CRLF" if crlf == lf and lf else ("LF" if crlf == 0 else "MIXED")
    if got != want: fail("%s 行尾 %s, 期望 %s" % (p, got, want))
    else: print("   %-46s %s" % (p, got))

print("5) sourceEntries 排除了 work/build/docs/scripts/obj")
cp = raw(".cproject")
for d0 in ["work", "build", "docs", "scripts", "obj"]:
    if ("|%s|" % d0) not in cp and not cp.count("|%s\"" % d0):
        fail("sourceEntries 没排除 " + d0)

print("\nRESULT: " + ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
