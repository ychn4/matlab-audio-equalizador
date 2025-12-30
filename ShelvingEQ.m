function [y, Hbands] = ShelvingEQ(x, fs, G, bandFreqs)
    
    Hbands = zeros(5,3,2);
    y = x;
    fmin = min(bandFreqs);
    fmax = max(bandFreqs);
    m = sqrt(fmin * fmax);
    
    for i = 1:5
        f0 = bandFreqs(i);
        g = G(i); 
        A = 10^(g/40); 
        w0 = 2*pi*f0/fs;
        cosw = cos(w0);
        sinw = sin(w0);
        Q = sqrt(2)/2; 
        alpha = sinw / (2*Q); 
        
        if f0 < m
            % Low-shelf 
            b0 =    A*( (A+1) - (A-1)*cosw + 2*sqrt(A)*alpha);
            b1 =  2*A*( (A-1) - (A+1)*cosw );
            b2 =    A*( (A+1) - (A-1)*cosw - 2*sqrt(A)*alpha);
            a0 =        (A+1) + (A-1)*cosw + 2*sqrt(A)*alpha;
            a1 =   -2*( (A-1) + (A+1)*cosw );
            a2 =        (A+1) + (A-1)*cosw - 2*sqrt(A)*alpha;
        elseif f0 > m
            % High-shelf 
            b0 =    A*( (A+1) + (A-1)*cosw + 2*sqrt(A)*alpha);
            b1 = -2*A*( (A-1) + (A+1)*cosw );
            b2 =    A*( (A+1) + (A-1)*cosw - 2*sqrt(A)*alpha);
            a0 =        (A+1) - (A-1)*cosw + 2*sqrt(A)*alpha;
            a1 =    2*( (A-1) - (A+1)*cosw );
            a2 =        (A+1) - (A-1)*cosw - 2*sqrt(A)*alpha;
        end
        
        b_shelv = [b0, b1, b2] / a0;
        a_shelv = [1, a1/a0, a2/a0];
        
        Hbands(i,:,1) = b_shelv;
        Hbands(i,:,2) = a_shelv;
        y = filter(b_shelv, a_shelv, y);
    end
end