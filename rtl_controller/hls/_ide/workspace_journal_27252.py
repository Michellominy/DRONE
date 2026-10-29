# 2026-09-09T10:30:49.861318
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

# 2026-09-09T10:30:49.861318
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.create_hls_component(name = "pid_controller",cfg_file = ["hls_config.cfg"],template = "empty_hls_component")

cfg = client.get_config_file(path="C:\Users\lomin\Documents\git\DRONE\rtl_controller\hls\pid_controller\hls_config.cfg")

cfg.set_values(key="syn.file", values=["pid_controller.cpp"])

vitis.dispose()

