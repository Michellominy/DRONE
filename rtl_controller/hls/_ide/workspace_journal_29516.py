# 2026-09-09T19:26:51.093233700
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

vitis.dispose()

