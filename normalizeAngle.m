function alpha = normalizeAngle(alpha, varargin)
%NORMALIZEANGLE  Normalize an angle value within a 2*PI interval


center = pi;
if ~isempty(varargin)
    center = varargin{1};
end

alpha = mod(alpha-center+pi, 2*pi) + center-pi;