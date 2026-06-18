"""
Scans all extension.toml files in the isaacsim exts directory, collects
every extension name that is referenced as a dependency but doesn't exist
in any known extension folder, then creates a minimal stub for each one.
"""
import os
import re
import sys

SITE_PACKAGES = os.path.join(sys.prefix, "lib", "python3.12", "site-packages")
ISAACSIM = os.path.join(SITE_PACKAGES, "isaacsim")

EXT_SEARCH_DIRS = [
    os.path.join(ISAACSIM, "exts"),
    os.path.join(ISAACSIM, "extscache"),
    os.path.join(ISAACSIM, "extsUser"),
    os.path.join(ISAACSIM, "kit", "exts"),
    os.path.join(ISAACSIM, "kit", "extscore"),
    os.path.join(ISAACSIM, "kit", "extsPhysics"),
]
STUB_DIR = os.path.join(ISAACSIM, "extsUser")

# Build set of known extension names (strip version suffixes like -1.2.3+xxx)
def get_known_extensions():
    known = set()
    for d in EXT_SEARCH_DIRS:
        if not os.path.isdir(d):
            continue
        for name in os.listdir(d):
            base = re.split(r"-\d", name)[0]
            known.add(base)
    return known

# Parse [dependencies] section from a .toml file (very simple parser)
DEP_SECTION_RE = re.compile(r'^\[dependencies\]', re.MULTILINE)
NEXT_SECTION_RE = re.compile(r'^\[', re.MULTILINE)
DEP_LINE_RE = re.compile(r'^"([^"]+)"\s*=')

def extract_deps(toml_text):
    deps = set()
    m = DEP_SECTION_RE.search(toml_text)
    if not m:
        return deps
    start = m.end()
    n = NEXT_SECTION_RE.search(toml_text, start)
    section = toml_text[start: n.start() if n else len(toml_text)]
    for line in section.splitlines():
        line = line.strip()
        if line.startswith("#"):
            continue
        dm = DEP_LINE_RE.match(line)
        if dm:
            deps.add(dm.group(1))
    return deps

def find_all_deps():
    all_deps = set()
    for d in EXT_SEARCH_DIRS:
        if not os.path.isdir(d):
            continue
        for root, dirs, files in os.walk(d):
            for f in files:
                if f == "extension.toml":
                    path = os.path.join(root, f)
                    try:
                        with open(path) as fp:
                            all_deps |= extract_deps(fp.read())
                    except Exception:
                        pass
    return all_deps

known = get_known_extensions()
all_deps = find_all_deps()
missing = sorted(all_deps - known)

print(f"Known extensions: {len(known)}")
print(f"Total referenced deps: {len(all_deps)}")
print(f"Missing extensions: {len(missing)}")

created = 0
for ext_name in missing:
    stub_path = os.path.join(STUB_DIR, ext_name, "config")
    toml_path = os.path.join(stub_path, "extension.toml")
    if os.path.exists(toml_path):
        continue
    os.makedirs(stub_path, exist_ok=True)
    with open(toml_path, "w") as f:
        f.write(f'[package]\nversion = "1.0.0"\ntitle = "{ext_name} stub"\n\n[dependencies]\n')
    print(f"  stub: {ext_name}")
    created += 1

print(f"\nCreated {created} new stubs.")
