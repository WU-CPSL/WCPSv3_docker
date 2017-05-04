function [delay] =delay_summary(result)

% 181 153 152 155 154 182 253 252 255 254; 180 157 108 175, in total 10 + 4 delivery status
result_temp_filled=[];
if(~isempty(result))
        result_num = result;
        [sz_x,sz_y] = size(result_num);
        delay=[1 88 88 88 88 88 88 88 88]; % 14 delivery status
        for i_rn = 1:sz_y
            if result_num(i_rn) >= 1
                delay(1, i_rn + 1) = 1;
            else
                delay(1, i_rn + 1) = delay(1, i_rn + 1);
            end
        end
else
    delay=[1 88 88 88 88 88 88 88 88];
end