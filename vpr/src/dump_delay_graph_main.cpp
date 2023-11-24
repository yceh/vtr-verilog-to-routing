#include "globals.h"
#include "read_blif.h"
#include "read_circuit.h"
#include "read_xml_arch_file.h"
int main(int argc, char** argv){
    t_arch Arch = t_arch();
    auto& device_ctx = g_vpr_ctx.mutable_device();
    XmlReadArch(argv[1],
                            true,
                            &Arch,
                            device_ctx.physical_tile_types,
                            device_ctx.logical_block_types);
    auto netlist=read_blif(e_circuit_format::AUTO, argv[1], Arch.models, Arch.model_library);
    netlist.compress();
    for(auto blockid:netlist.blocks()){
        auto block=netlist
    }
}