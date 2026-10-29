# 2026-10-17T23:26:43.548738500
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

vitis.dispose()

