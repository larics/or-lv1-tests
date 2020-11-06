function err = test_closest_q(cq)
% Test calculation of closest feasible joint configuration
%    ERR = TEST_CLOSEST_Q(@CQ) evaluates the function closest_q
%    
%    ERR is percentage of failure cases
%    @CQ is a handle to the closest_q function

load('./data/q_matrices.mat');
load('./data/qs.mat');
load('./data/qinits.mat');

error_count = 0;

for i=1:size(q_matrices,2)
  q_goal = qs{i};
  q_student = cq(qinits{i},q_matrices{i});
  
  if any(q_goal)
    if any(q_student)
      if norm(q_goal-q_student, 'inf') > 1e-6
        error_count = error_count + 1; 
      end
    else
      error_count = error_count + 1; 
    end
  elseif any(q_student)
    error_count = error_count + 1; 
  end
end

err = error_count/size(q_matrices,2)*100;
disp('--- Testing closest_q ---')
disp(['closest_q error rate: ' mat2str(err) '%'])