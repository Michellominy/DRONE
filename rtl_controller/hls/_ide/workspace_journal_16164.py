# 2026-10-12T12:23:23.944077500
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="pid_controller")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

