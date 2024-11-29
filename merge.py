import sys
import os
import json

def merge_bin():
    with open("build/flasher_args.json") as f:
        # 加载json文件
        data = json.load(f)

        # 提取flash设置
        flash_mode = data["flash_settings"]["flash_mode"]
        flash_size = data["flash_settings"]["flash_size"]
        flash_freq = data["flash_settings"]["flash_freq"]

        # 提取flash文件及其偏移量
        flash_files = data["flash_files"]

        # 构建合并命令
        # esptool --chip esp32s3 merge_bin -o merged-flash.bin --flash_mode dio --flash_size 16MB --flash_freq 80m 
        command = f"esptool --chip esp32s3 merge_bin -o merged.bin --flash_mode {flash_mode} --flash_size {flash_size} --flash_freq {flash_freq}"
        # 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xd000 ota_data_initial.bin 0x10000 srmodels/srmodels.bin 0x200000 xiaozhi.bin
        for offset, file in flash_files.items():
            command += f" {offset} {file}"

        # 执行合并命令
        # 先打开build目录，然后执行命令
        os.chdir("build")
        if os.system(command) != 0:
            print("merge bin failed")
            sys.exit(1)

if __name__ == "__main__":
    merge_bin()
