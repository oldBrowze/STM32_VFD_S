Import("env")

#print(env.Dump())

env.Append(
  LINKFLAGS=[
    "-mcpu=cortex-m4",
    "-mthumb",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
    #"-specs=nosys.specs",
    #"-specs=nano.specs",
    #"-nostartfiles",
    "-lstdc++",
    "-lc",
    "-lm",
    "-lnosys",
    "-Wl,--gc-sections",
    #"-Xlinker -print-memory-usage -Xlinker"
  ]
)