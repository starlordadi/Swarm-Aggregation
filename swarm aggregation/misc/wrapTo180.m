function [out] = wrapTo180(in)
%NORM_ANGLE Normalizes input angles to range [-180..180]
%   Input vector is expected to be degree angles that are unwrapped.
    out = mod(in + pi, 2*pi) - pi;
end