function loop_params = dsst_c_init(im, params)

% parameters - independent
padding = params.padding;                         	%extra area surrounding the target
output_sigma_factor = params.output_sigma_factor;	%spatial bandwidth (proportional to target)
loop_params.lambda = params.lambda;
loop_params.learning_rate = params.learning_rate;
nScales = params.number_of_scales;
scale_step = params.scale_step;
scale_sigma_factor = params.scale_sigma_factor;
scale_model_max_area = params.scale_model_max_area;

loop_params.pos = floor(params.init_pos);
target_sz = floor(params.wsize);

init_target_sz = target_sz;

% target size att scale = 1
loop_params.base_target_sz = target_sz;

% window size, taking padding into account
loop_params.sz = floor(loop_params.base_target_sz * (1 + padding));

% desired translation filter output (gaussian shaped), bandwidth
% proportional to target size
output_sigma = sqrt(prod(loop_params.base_target_sz)) * output_sigma_factor;
[rs, cs] = ndgrid((1:loop_params.sz(1)) - floor(loop_params.sz(1)/2), ...
    (1:loop_params.sz(2)) - floor(loop_params.sz(2)/2));
y = exp(-0.5 * (((rs.^2 + cs.^2) / output_sigma^2)));
loop_params.yf = single(fft2(y));


% desired scale filter output (gaussian shaped), bandwidth proportional to
% number of scales
scale_sigma = nScales/sqrt(33) * scale_sigma_factor;
ss = (1:nScales) - ceil(nScales/2);
ys = exp(-0.5 * (ss.^2) / scale_sigma^2);
loop_params.ysf = single(fft(ys));

% store pre-computed translation filter cosine window
loop_params.cos_window = single(hann(loop_params.sz(1)) * hann(loop_params.sz(2))');

% store pre-computed scale filter cosine window
if mod(nScales,2) == 0
    scale_window = single(hann(nScales+1));
    loop_params.scale_window = scale_window(2:end);
else
    loop_params.scale_window = single(hann(nScales));
end;

% scale factors
ss = 1:nScales;
loop_params.scaleFactors = scale_step.^(ceil(nScales/2) - ss);

% compute the resize dimensions used for feature extraction in the scale
% estimation
scale_model_factor = 1;
if prod(init_target_sz) > scale_model_max_area
    scale_model_factor = sqrt(scale_model_max_area/prod(init_target_sz));
end
loop_params.scale_model_sz = floor(init_target_sz * scale_model_factor);

% loop_params.currentScaleFactor = 1;


% find maximum and minimum scales
loop_params.min_scale_factor = scale_step ^ ceil(log(max(5 ./ loop_params.sz)) / log(scale_step));
loop_params.max_scale_factor = scale_step ^ floor(log(min([size(im,1) size(im,2)] ./ ...
    loop_params.base_target_sz)) / log(scale_step));

%pos sz currentScaleFactor cos_window lambda base_target_sz scaleFactors
%scale_window scale_model_sz min_scale_factor max_scale_factor
%learning_rate target_sz initial yf ysf
