
% Init matlab. 
clearvars
close all
clc 

% Init. 
Nel = 8; 
lsb_att = [0.5 1 2 4 8 16]; 
nof_bits = [7 6 5 4 3 2]; 
lsb_phase = 2*pi ./ 2.^nof_bits; 

% Calculate error standard deviation. 
sa = 10.^(lsb_att/40)-10.^(lsb_att/40); 
sp = lsb_phase; 
sigma_e_sq = sa.^2/12 + sp.^2/12; 

% Calculate expected sidelobe level. 
sll = 10*log10(sigma_e_sq) - 10*log10(Nel);

% Print table with results. 
nof_q = length(lsb_att); 
for i_q = 1:nof_q
    fprintf('%d PS bits, %.2f dB att resolution, SLL = %.2f dB.\n', nof_bits(i_q), lsb_att(i_q), sll(i_q)); 
end 