# 2026-10-24T13:41:28.424239
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

vitis.dispose()

