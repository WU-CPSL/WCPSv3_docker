%%Matlab function for handling socket communications between Matlab and
% Python event server (on Docker container)

function result = SocketHandle(rssi)
    t = tcpip('192.168.99.100', 55000);
    fopen(t);
    message =strcat(num2str(rssi));
    fwrite(t,message);
    
    amount_received = 0;	
	amount_expected = length(message);
    
    while amount_received < amount_expected
        bytes = fread(t, [1, t.BytesAvailable]);
        if length(bytes) > 0
            amount_received = amount_received + length(char(bytes));
            result = char(bytes);
        end
    end
    
    fclose(t);    
end