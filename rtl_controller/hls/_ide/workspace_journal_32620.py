# 2026-09-11T12:12:42.814779900
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="mixer_module")
comp.run(operation="SYNTHESIS")

comp = client.get_component(name="sensor_fusion")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

comp = client.get_component(name="pid_controller")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

comp = client.get_component(name="mixer_module")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

