from crc import Calculator, Configuration

crc16_config = Configuration(
    width=16,
    polynomial=0x1021,
    init_value=0xffff,
    final_xor_value=0x0000,
    reverse_input=False,
    reverse_output=False,
)
crc32_config = Configuration(
    width=32,
    polynomial=0x04c11db7,
    init_value=0xffffffff,
    final_xor_value=0xffffffff,
    reverse_input=True,
    reverse_output=True,
)

crc16 = Calculator(crc16_config, optimized=True)
crc32 = Calculator(crc32_config, optimized=True)
