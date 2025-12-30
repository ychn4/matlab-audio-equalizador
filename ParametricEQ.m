function [y, Hbands] = ParametricEQ(x, fs, G, bandFreqs, Q)
    
    Hbands = zeros(5,3,2);
    y = x;

    for i = 1:5 
        f0 = bandFreqs(i);
        g = G(i);
        
        A = sqrt(10^(g/20));
        w = 2*pi*f0/fs;
        cosw = cos(w);
        sinw = sin(w);
        alpha = sinw/(2*Q);

        b0 = 1 + alpha*A;
        b1 = -2*cosw;
        b2 = 1 - alpha*A;
        a0 = 1 + alpha/A;
        a1 = -2*cosw;
        a2 = 1 - alpha/A;

        b_param = [b0 b1 b2] / a0;
        a_param = [1 a1/a0 a2/a0];
        
        Hbands(i,:,1) = b_param; 
        Hbands(i,:,2) = a_param;
        y = filter(b_param, a_param, y); 
    end 
end