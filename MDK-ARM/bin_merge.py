"""
This is a script to merage the compiled file (ROM and QSPI)
"""

ROM_BIN_PATH = r"myprog.bin/ER_IROM2"
ROM_BIN_SPACE = 0x100000
QSPI_BIN_PATH = r"myprog.bin/ER_QSPI"
QSPI_BIN_SPACE = 0xFE0000
OUTPUT_FILE_NAME = r"myprog.bin/output.bin"

with open(ROM_BIN_PATH, "rb") as rom_bin:
    rom_bin_file_content = rom_bin.read()
    rom_bin_file_size = len(rom_bin_file_content)

with open(QSPI_BIN_PATH, "rb") as qspi_bin:
    qspi_bin_file_content = qspi_bin.read()
    # qspi_bin_file_size = len(qspi_bin_file_content)

with open(OUTPUT_FILE_NAME, "wb") as output:
    rom_bin_zero_content = [0]*(ROM_BIN_SPACE - rom_bin_file_size)
    rom_bin_zero_content = bytearray(rom_bin_zero_content)
    output.write(rom_bin_file_content)
    output.write(rom_bin_zero_content)
    output.write(qspi_bin_file_content)
