load('data_ode.mat');
odedata = iddata(out.output.Data,out.input.Data,0.02);
w = idWaveletNetwork();
sys = nlhw(odedata,[2 2 1],'idWaveletNetwork' ,w);
compare(odedata,sys);