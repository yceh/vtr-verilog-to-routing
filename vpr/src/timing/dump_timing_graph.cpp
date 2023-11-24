#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include "AnalysisDelayCalculator.h"
#include "DelayType.h"
#include "atom_netlist.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <fcntl.h>
#include "atom_netlist_fwd.h"
#include "delay_graph.capnp.h"
#include "logic_types.h"
#include "physical_types.h"
#include "vtr_vector_map.h"
void dump_timing_graph(std::shared_ptr<AnalysisDelayCalculator> delay_calc, const AtomNetlist& nl) {
    ::capnp::MallocMessageBuilder message;

    Graph::Builder gout = message.initRoot<Graph>();
    //Number all the output pins first
    vtr::vector_map<AtomPinId, size_t> out_pin_id_map;
    out_pin_id_map.resize(nl.pins().size(), SIZE_MAX);
    // vtr::vector_map<AtomBlockId, std::pair<size_t,size_t>> block_pin_id_range;
    //block_pin_id_range.resize(nl.blocks().size());
    size_t cur_out_idx = 0;
    for (auto blk_id : nl.blocks()) {
        if (blk_id.INVALID()) {
            continue;
        }
        //auto start_pin_id=cur_out_idx;
        for (auto port_id : nl.block_ports(blk_id)) {
            if (port_id.INVALID()) {
                continue;
            }
            auto port_model = nl.port_model(port_id);
            if (port_model->dir == OUT_PORT) {
                for (auto pin_id : nl.port_pins(port_id)) {
                    if (pin_id.INVALID()) {
                        continue;
                    }
                    out_pin_id_map[pin_id] = cur_out_idx++;
                }
            }
        }
        //block_pin_id_range[blk_id]=std::make_pair(start_pin_id,cur_out_idx);
    }

    gout.initVertices(cur_out_idx);
    auto vertices = gout.getVertices();

    for (auto blk_id : nl.blocks()) {
        if (blk_id.INVALID()) {
            continue;
        }
        struct Each_Sink_Src {
            std::vector<std::pair<size_t, float>> delays;
            const t_pb_graph_pin* sink_pin;
            AtomPinId id;
        };
        std::vector<Each_Sink_Src> sinks_delay;
        for (auto port_id : nl.block_ports(blk_id)) {
            if (port_id.INVALID()) {
                continue;
            }
            auto port_model = nl.port_model(port_id);
            if (port_model->dir == OUT_PORT) {
                for (auto pin_id : nl.port_pins(port_id)) {
                    if (pin_id.INVALID()) {
                        continue;
                    }
                    Each_Sink_Src each_sink_src;
                    each_sink_src.id = pin_id;
                    each_sink_src.sink_pin = delay_calc->atom_delay_calc_.find_pb_graph_pin(pin_id);
                    sinks_delay.push_back(std::move(each_sink_src));
                }
            }
        }
        struct In_Pin_Info{
            size_t driver_pin_id;
            float comb_delay;
            int reg_cnt;
        };
        std::vector<In_Pin_Info> in_pin_info;
        for (auto port_id : nl.block_ports(blk_id)) {
            if (port_id.INVALID()) {
                continue;
            }
            auto port_model = nl.port_model(port_id);
            //Ignore clock and other async signals
            if(port_model->is_clock||port_model->is_non_clock_global){
                continue;
            }
            if (port_model->dir == IN_PORT) {
                for (auto pin_id : nl.port_pins(port_id)) {
                    if (pin_id.INVALID()) {
                        continue;
                    }
                    In_Pin_Info info;
                    info.reg_cnt=0;
                    info.comb_delay=0;
                    auto src_gpin = delay_calc->atom_delay_calc_.find_pb_graph_pin(pin_id);
                    if(src_gpin->type==PB_PIN_SEQUENTIAL){
                        if(!std::isnan(src_gpin->tsu)){
                            info.reg_cnt++;
                            info.comb_delay+=src_gpin->tsu;
                        }
                        if(!std::isnan(src_gpin->tco_max)){
                            info.reg_cnt++;
                            info.comb_delay+=src_gpin->tco_max;
                        }
                    }
                    auto nid=nl.pin_net(pin_id);
                    //ignore unconnected pins
                    if(nid.INVALID()){
                        continue;
                    }
                    auto driver=nl.net_driver(nid);
                    if(driver.INVALID()){
                        continue;
                    }
                    info.driver_pin_id=out_pin_id_map[driver];
                    info.comb_delay+=delay_calc->atom_net_delay(driver, pin_id, DelayType::MAX);
                    
                    //Add this fanin to the sinks
                    for (int i = 0; i < src_gpin->num_pin_timing; ++i) {
                        const t_pb_graph_pin* timing_sink_gpin = src_gpin->pin_timing[i];
                        for (auto& sinks : sinks_delay) {
                            if (timing_sink_gpin == sinks.sink_pin) {
                                sinks.delays.push_back(std::make_pair(in_pin_info.size(), src_gpin->pin_timing_del_max[i]));
                            }
                        }
                    }
                    in_pin_info.push_back(std::move(info));

                }
            }
        }
        size_t out_pin_idx=0;
        for(const auto& out_pins:sinks_delay){
            auto out_vertex=vertices[out_pin_id_map[out_pins.id]];
            out_vertex.setComponentId(size_t(blk_id));
            out_vertex.setOutportId(out_pin_idx++);
            out_vertex.setId(out_pin_id_map[out_pins.id]);
            if(out_pins.sink_pin->type==PB_PIN_SEQUENTIAL){
                //Couldn't find path from input pin to sequential output pin, so assume all input pins feed into the sequential input pin
                out_vertex.initFanins(in_pin_info.size());
                auto sinks=out_vertex.getFanins();
                auto delay_iter=out_pins.delays.begin();
                float out_delay=0;
                int out_reg_cnt=0;
                if(!std::isnan(out_pins.sink_pin->tsu)){
                    out_reg_cnt++;
                    out_delay+=out_pins.sink_pin->tsu;
                }
                if(!std::isnan(out_pins.sink_pin->tco_max)){
                    out_reg_cnt++;
                    out_delay+=out_pins.sink_pin->tco_max;
                }
                for(size_t in_pin_idx=0;in_pin_idx<in_pin_info.size();++in_pin_idx){
                    auto delay=out_delay+in_pin_info[in_pin_idx].comb_delay;
                    auto reg_cnt=out_reg_cnt+in_pin_info[in_pin_idx].reg_cnt;
                    if(delay_iter!=out_pins.delays.end()&&delay_iter->first==in_pin_idx){
                        delay+=delay_iter->second;
                        delay_iter++;
                    }
                    sinks[in_pin_idx].setDriverId(in_pin_info[in_pin_idx].driver_pin_id);
                    sinks[in_pin_idx].setCombDelay(delay*1e12);
                    sinks[in_pin_idx].setRegCnt(reg_cnt/2);
                }
            }else {
                //Combinational output pin, so only add inputs aready in comb_delay field
                out_vertex.initFanins(out_pins.delays.size());
                auto sinks=out_vertex.getFanins();
                for (size_t fanin_idx=0; fanin_idx<out_pins.delays.size(); fanin_idx++) {
                    auto in_pin_idx=out_pins.delays[fanin_idx].first;
                    auto delay=in_pin_info[in_pin_idx].comb_delay+out_pins.delays[fanin_idx].second;
                    sinks[in_pin_idx].setDriverId(in_pin_info[in_pin_idx].driver_pin_id);
                    sinks[fanin_idx].setCombDelay(delay*1e12);
                    sinks[fanin_idx].setRegCnt(in_pin_info[in_pin_idx].reg_cnt/2);
                }                
            }
        }
    }

    auto fd=open("delay_graph_dump", O_WRONLY|O_CREAT|O_TRUNC, 0666);
    capnp::writePackedMessageToFd(fd,message);

}