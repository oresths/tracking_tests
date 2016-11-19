function [position_out, state_out] = dsst_c(im, state_in, loop_params, initial)

%pos sz currentScaleFactor cos_window lambda base_target_sz scaleFactors
%scale_window scale_model_sz min_scale_factor max_scale_factor
%learning_rate target_sz initial yf ysf

sz = loop_params.sz;
cos_window = loop_params.cos_window;
lambda = loop_params.lambda;
base_target_sz = loop_params.base_target_sz;
scaleFactors = loop_params.scaleFactors;
scale_window = loop_params.scale_window;
scale_model_sz = loop_params.scale_model_sz;
min_scale_factor = loop_params.min_scale_factor;
max_scale_factor = loop_params.max_scale_factor;
learning_rate = loop_params.learning_rate;
yf = loop_params.yf;
ysf = loop_params.ysf;

if initial == 1
    pos = loop_params.pos;
    state_out.pos = pos;
    currentScaleFactor = 1;
    state_out.currentScaleFactor = currentScaleFactor;
else    
    pos = state_in.pos;
    currentScaleFactor = state_in.currentScaleFactor;
    hf_num = state_in.hf_num;
    hf_den = state_in.hf_den;
    sf_num = state_in.sf_num;
    sf_den =state_in.sf_den;
end
  
%pos currentScaleFactor hf_num hf_den sf_num sf_den

if initial == 0

    % extract the test sample feature map for the translation filter
    xt = get_translation_sample(im, pos, sz, currentScaleFactor, cos_window);

    % calculate the correlation response of the translation filter
    xtf = fft2(xt);
    response = real(ifft2(sum(hf_num .* xtf, 3) ./ (hf_den + lambda)));

    % find the maximum translation response
    [row, col] = find(response == max(response(:)), 1);

    % update the position
    pos = pos + round((-sz/2 + [row, col]) * currentScaleFactor);
    state_out.pos = pos;

    % extract the test sample feature map for the scale filter
    xs = get_scale_sample(im, pos, base_target_sz, currentScaleFactor * scaleFactors, scale_window, scale_model_sz);

    % calculate the correlation response of the scale filter
    xsf = fft(xs,[],2);
    scale_response = real(ifft(sum(sf_num .* xsf, 1) ./ (sf_den + lambda)));

    % find the maximum scale response
    recovered_scale = find(scale_response == max(scale_response(:)), 1);

    % update the scale
    currentScaleFactor = currentScaleFactor * scaleFactors(recovered_scale);
    state_out.currentScaleFactor = currentScaleFactor;
    if currentScaleFactor < min_scale_factor
        currentScaleFactor = min_scale_factor;
    elseif currentScaleFactor > max_scale_factor
        currentScaleFactor = max_scale_factor;
    end
end

% extract the training sample feature map for the translation filter
xl = get_translation_sample(im, pos, sz, currentScaleFactor, cos_window);

% calculate the translation filter update
xlf = fft2(xl);
new_hf_num = bsxfun(@times, yf, conj(xlf));
new_hf_den = sum(xlf .* conj(xlf), 3);

% extract the training sample feature map for the scale filter
xs = get_scale_sample(im, pos, base_target_sz, currentScaleFactor * scaleFactors, scale_window, scale_model_sz);

% calculate the scale filter update
xsf = fft(xs,[],2);
new_sf_num = bsxfun(@times, ysf, conj(xsf));
new_sf_den = sum(xsf .* conj(xsf), 1);


if initial == 1
    % first frame, train with a single image
    state_out.hf_den = new_hf_den;
    state_out.hf_num = new_hf_num;

    state_out.sf_den = new_sf_den;
    state_out.sf_num = new_sf_num;
else
    % subsequent frames, update the model
    state_out.hf_den = (1 - learning_rate) * hf_den + learning_rate * new_hf_den;
    state_out.hf_num = (1 - learning_rate) * hf_num + learning_rate * new_hf_num;
    state_out.sf_den = (1 - learning_rate) * sf_den + learning_rate * new_sf_den;
    state_out.sf_num = (1 - learning_rate) * sf_num + learning_rate * new_sf_num;
end

% calculate the new target size
target_sz = floor(base_target_sz * currentScaleFactor);

%save position
position_out = [pos target_sz];
    