# 2026-10-18T20:08:53.930780300
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="mixer_module")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

