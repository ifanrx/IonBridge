#ifndef H_FPGA_
#define H_FPGA_

#define FPGA_BITSTREAM_BUFSZ 4096

void fpga_init(const char* variant = nullptr);

#endif  // !H_FPGA_
