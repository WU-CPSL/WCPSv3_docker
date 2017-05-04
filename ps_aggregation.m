function [resultz] = ps_aggregation (call_count, rssi)

resultz = zeros(call_count, 8);

for i_pa = 1 : call_count

    result = SocketHandle(rssi);
    
    resultz(i_pa, :) = str2num(result);
end