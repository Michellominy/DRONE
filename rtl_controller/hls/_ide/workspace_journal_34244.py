# 2026-09-08T20:37:12.400490300
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="sensor_fusion")
comp.run(operation="SYNTHESIS")

comp.run(operation="SYNTHESIS")

vitis.dispose()

