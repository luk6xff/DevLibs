Import('env')
from os.path import join, realpath

# Pick the platform implementation selected with -D PLATFORM=<arduino|mbed|stm32cube>
for item in env.get("CPPDEFINES", []):
    if isinstance(item, tuple) and item[0] == "PLATFORM":
        env.Append(CPPPATH=[realpath(join("platform", item[1]))])
        env.Replace(SRC_FILTER=["+<*>", "-<platform>", "+<%s>" % join("platform", item[1])])
        break
