function [err] = test_transform_w(tfw)
% Test transformation of the configuration
%    [ERR] = TEST_TRANSFORM_W(N,@TFW) Tests the transformation
%    of configuration. 
%    
%    ERR is percentage of failed cases
%    @TFW is a handle to the transformation function

load('./data/bases.mat');
load('./data/poses.mat');
load('./data/transforms.mat');

failed_count = 0;

for i=1:size(bases,2)
        
    transform = [0 -1 0 0; -1 0 0 0; 0 0 -1 0; 0 0 0 1];
    w_global = poses{i} * transform;
    translate = [1 0 0 0; 0 1 0 0; 0 0 1 0.01231; 0 0 0 1];
    w_global = w_global * translate;
    transform_w_student = tfw(w_global, bases{i});
    diff = transform_w_student - transforms{i};
    if norm(diff,'inf') > 1e-6
    failed_count = failed_count + 1;
    end

end  
err = failed_count/size(bases,2)*100;
disp('--- Testing transform_w ---')
disp(['transform_w error rate: ' mat2str(err) '%'])