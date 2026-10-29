# 2026-09-11T15:00:36.184579500
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

cfg = client.get_config_file(path="C:\Users\lomin\Documents\git\DRONE\rtl_controller\hls\sensor_fusion\hls_config.cfg")

cfg.set_values(key="tb.file", values=["sensor_fusion_tb.cpp"])

cfg.set_values(key="syn.file", values=["sensor_fusion.cpp", "sensor_fusion.h"])

comp = client.get_component(name="sensor_fusion")
comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

cfg.set_values(key="tb.file", values=["sensor_fusion_tb.cpp", "test_data.csv"])

cfg.set_values(key="tb.file", values=["sensor_fusion_tb.cpp", "test_data.csv", "gen_test_data.py"])

comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

cfg.set_values(key="tb.file", values=["sensor_fusion_tb.cpp", "test_data.csv", "test_bench_gen.py"])

comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

comp.run(operation="C_SIMULATION")

comp.run(operation="SYNTHESIS")

comp.run(operation="PACKAGE")

vitis.dispose()

