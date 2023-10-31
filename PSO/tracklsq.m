function F = tracklsq(pid)
         % Track the output of optsim to a signal of 1
         % Input: [Kp, Ki, Kd]
         % Output: []
        
         % Variables a1 and a2 are shared with RUNTRACKLSQ
         Kp = pid(1);
         Ki = pid(2);
         Kd = pid(3);
         
%          sprintf('The value of interation Kp= %3.0f, Ki= %3.0f, Kd= %3.0f', pid(1),pid(2),pid(3))
         % Compute function value
         simopt = simset('solver','ode5','SrcWorkspace','Current','DstWorkspace','Current');  % Initialize sim options
         [tout,xout,yout] = sim('optsim1',[0 100],simopt);
         e=yout-1 ;  % compute the error 
         sys_overshoot=max(yout)-1; % compute the overshoot
         
         alpha=1;
         beta=1;
         F = e2*beta+sys_overshoot*alpha; % weighted performance evaluation    
    end