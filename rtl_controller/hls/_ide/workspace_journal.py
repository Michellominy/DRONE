# 2026-06-28T17:00:41.566861100
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.create_hls_component(name = "gyro_fusion",cfg_file = ["gyro_fusion_config.cfg"],template = "empty_hls_component")

cfg = client.get_config_file(path="C:\Users\lomin\Documents\git\DRONE\rtl_controller\hls\gyro_fusion\gyro_fusion_config.cfg")

cfg.set_values(key="syn.file", values=["gyro_fusion.cpp"])

vitis.dispose()

