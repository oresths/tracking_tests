
% run_tracker.m

close all;
% clear all;

%choose the path to the videos (you'll be able to choose one with the GUI)
base_path = 'sequences/';

%parameters according to the paper
params.padding = 1.0;         			% extra area surrounding the target
params.output_sigma_factor = 1/16;		% standard deviation for the desired translation filter output
params.scale_sigma_factor = 1/4;        % standard deviation for the desired scale filter output
params.lambda = 1e-2;					% regularization weight (denoted "lambda" in the paper)
params.learning_rate = 0.025;			% tracking model learning rate (denoted "eta" in the paper)
params.number_of_scales = 33;           % number of scale levels (denoted "S" in the paper)
params.scale_step = 1.02;               % Scale increment factor (denoted "a" in the paper)
params.scale_model_max_area = 512;      % the maximum size of scale examples

visualization = 1;

%ask the user for the video
video_path = choose_video(base_path);
if isempty(video_path), return, end  %user cancelled
[img_files, pos, target_sz, ground_truth, video_path] = ...
	load_video_info(video_path);

params.init_pos = floor(pos) + floor(target_sz/2);
params.wsize = floor(target_sz);
params.img_files = img_files;
params.video_path = video_path;

% to calculate precision
positions = zeros(numel(img_files), 4);
% to calculate FPS
time = 0;

im = imread([video_path img_files{1}]);
loop_params = dsst_c_init(im, params);

tic
[position, state] = dsst_c(im, 0, loop_params, 1);
time = time + toc;
positions(1,:) = position;

num_frames = numel(img_files);

for i=1:num_frames-1
    im = imread([video_path img_files{i+1}]);
    tic
    [position, state] = dsst_c(im, state, loop_params, 0);

    time = time + toc;
    positions(i+1,:) = position;
    
    %visualization
    if visualization == 1
        pos = position(1:2);
        target_sz = position(3:4);        
        rect_position = [pos([2,1]) - target_sz([2,1])/2, target_sz([2,1])];
        if i == 1  %first frame, create GUI
            figure('NumberTitle','off', 'Name',['Tracker - ' video_path]);
            im_handle = imshow(uint8(im), 'Border','tight', 'InitialMag', 100 + 100 * (length(im) < 500));
            rect_handle = rectangle('Position',rect_position, 'EdgeColor','g');
            text_handle = text(10, 10, int2str(i));
            set(text_handle, 'color', [0 1 1]);
        else
            try  %subsequent frames, update GUI
                set(im_handle, 'CData', im)
                set(rect_handle, 'Position', rect_position)
                set(text_handle, 'string', int2str(i));
            catch
                return
            end
        end

        drawnow
    %         pause
    end
end
fps = num_frames / time;

% calculate precisions
[distance_precision, PASCAL_precision, average_center_location_error] = ...
    compute_performance_measures(positions, ground_truth);

fprintf('Center Location Error: %.3g pixels\nDistance Precision: %.3g %%\nOverlap Precision: %.3g %%\nSpeed: %.3g fps\n', ...
    average_center_location_error, 100*distance_precision, 100*PASCAL_precision, fps);
