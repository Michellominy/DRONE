# 2026-05-07T15:32:26.905156400
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

cfg = client.get_config_file(path="C:\Users\lomin\Documents\git\DRONE\hls\pid_controller\hls_config.cfg")

cfg.set_values(key="syn.file", values=["pid.cpp"])

cfg = client.get_config_file(path="/c:/Users/lomin/Documents/git/DRONE/hls/pid_controller/hls_config.cfg")

cfg.set_value(section="hls", key="syn.top", value="hardware_pid_engine")

comp = client.get_component(name="pid_controller")
comp.run(operation="SYNTHESIS")

vitis.dispose()

