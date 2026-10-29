# 2026-09-08T20:18:55.304805
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.create_hls_component(name = "sensor_fusion",cfg_file = ["hls_config.cfg"],template = "empty_hls_component")

cfg = client.get_config_file(path="C:\Users\lomin\Documents\git\DRONE\rtl_controller\hls\sensor_fusion\hls_config.cfg")

cfg.set_values(key="syn.file", values=["sensor_fusion.cpp"])

comp = client.get_component(name="sensor_fusion")
comp.run(operation="SYNTHESIS")

cfg = client.get_config_file(path="/c:/Users/lomin/Documents/git/DRONE/rtl_controller/hls/sensor_fusion/hls_config.cfg")

cfg.set_value(section="hls", key="syn.top", value="sensor_fusion")

comp.run(operation="SYNTHESIS")

comp.run(operation="SYNTHESIS")

comp.run(operation="C_SIMULATION")

comp.run(operation="SYNTHESIS")

vitis.dispose()

