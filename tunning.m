clear
close
% Tunning of parameters based on try and error
sigma_1 = 0.001:0.001:0.009;
sigma_2 = 0.0001:0.0001:0.0009;
tic
while true
    for i_sig1 = 1:size(sigma_1,2)
        sig_1 = sigma_1(i_sig1);
        for i_sig2 = 1:size(sigma_2,2)
            sig_2 = sigma_2(i_sig2);
            create_mountain_car
            main_p2_mc_rl
            if FLAG == 1 
                break
            end
        end
        if FLAG == 1 
            break
        end
    end
    if FLAG == 1 
        break
    end
end
toc