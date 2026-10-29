# 2026-09-09T13:43:41.518692400
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.get_component(name="sensor_fusion")
comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

comp = client.get_component(name="pid_controller")
comp.run(operation="SYNTHESIS")

cfg = client.get_config_file(path="/c:/Users/lomin/Documents/git/DRONE/rtl_controller/hls/pid_controller/hls_config.cfg")

cfg.set_value(section="hls", key="syn.top", value="pid_controller")

comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

comp.run(operation="PACKAGE")

comp.run(operation="PACKAGE")

comp = client.get_component(name="sensor_fusion")
comp.run(operation="PACKAGE")

comp = client.create_hls_component(name = "mixer_module",cfg_file = ["hls_config.cfg"],template = "empty_hls_component")

cfg = client.get_config_file(path="C:\Users\lomin\Documents\git\DRONE\rtl_controller\hls\mixer_module\hls_config.cfg")

cfg.set_values(key="syn.file", values=["mixer_module.cpp"])

comp = client.get_component(name="mixer_module")
comp.run(operation="SYNTHESIS")

cfg = client.get_config_file(path="/c:/Users/lomin/Documents/git/DRONE/rtl_controller/hls/mixer_module/hls_config.cfg")

cfg.set_value(section="hls", key="syn.top", value="mixer_module")

comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

