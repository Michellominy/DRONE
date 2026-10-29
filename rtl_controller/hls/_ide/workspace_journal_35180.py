# 2026-10-17T22:11:47.199313700
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="mixer_module")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

