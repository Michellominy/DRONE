# 2026-10-03T13:20:22.564899500
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

vitis.dispose()

