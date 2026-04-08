import sys
def get_mem_config():
    flash_start = 0x08000000
    flash_size  = 0x00100000
    ram_start   = 0x20000000
    ram_size    = 0x00020000
    print("FLASH_ADD={}".format(hex(flash_start)))
    print("FLASH_SIZE={}".format(hex(flash_size)))
    print("RAM_ADD={}".format(hex(ram_start)))
    print("RAM_SIZE={}".format(hex(ram_size)))

    print(sys.argv[2])
    print(sys.argv[3])
    print(sys.argv[4])
    print(sys.argv[5])

if __name__ == "__main__":
    get_mem_config()
