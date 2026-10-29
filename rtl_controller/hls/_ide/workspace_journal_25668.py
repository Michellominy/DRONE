# 2026-09-11T13:51:29.501920100
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="sensor_fusion")
comp.run(operation="SYNTHESIS")

comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

