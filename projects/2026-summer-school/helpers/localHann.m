function window = localHann(nSamples)
% LOCALHANN Toolbox-independent periodic Hann window.
%
%   window = localHann(nSamples) returns an nSamples-by-1 periodic Hann window.

    if nSamples == 1
        window = 1;
    else
        window = 0.5 - 0.5 * cos(2 * pi * (0:nSamples - 1).' / nSamples);
    end
end
