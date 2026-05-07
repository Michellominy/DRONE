# 2026-05-07T15:24:01.060228
import vitis

client = vitis.create_client()
client.set_workspace(path="hls")

comp = client.create_hls_component(name = "pid_controller",cfg_file = ["hls_config.cfg"],template = "empty_hls_component")

