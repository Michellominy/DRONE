# 2026-09-11T14:33:26.074150700
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="sensor_fusion")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

