# 2026-05-07T16:34:00.592948
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="pid_controller")
comp.run(operation="PACKAGE")

vitis.dispose()

