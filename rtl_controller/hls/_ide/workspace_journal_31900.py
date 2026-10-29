# 2026-10-03T23:07:21.622246200
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="pid_controller")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

