function [err,q] = test_nao_rarm_kinematics(n,dkf,ikf)
% Test an inverse kinematics solver
%    [ERR,Q] = TEST_NAO_RARM_KINEMATICS(N,@DKF,@IKF) Tests the kinematics
%    solver in N points. 
%    
%    ERR is the vector of euclidean distances between initial and computed
%    tool configurations (in cases when the tested function returns several
%    possible solutions, the solution with the smallest error is
%    considered).
%    DKF is a handle to the direct kinematics solver (necessary for
%    testing).
%    IKF is a handle to the inverse kinematics solver.

l = [15,105,55.95,57.50,12.31];
q_min = [-119.5,-76, 0, 2, -104.5]*pi/180+10*eps;
q_max = [119.5,18,0,88.5,104.5]*pi/180-10*eps;


disp(' ')
disp('--- Testing infeasible point ---')
T_in = eye(4);
T_in(1:3,4) = [10000;10000;10000];
q_out = ikf(T_in,l);
disp(['Configuration for infeasible point: ' mat2str(q_out)])

% Test some critical points

disp(' ')
disp('--- Testing critical point q = [0,0,0,0.035,0] ---')
q_in = [0,0,0,0.035,0];
w_in = dkf(q_in,l);
q_out = ikf(w_in,l);
disp(['Configuration in critical point: ' mat2str(q_out)])

disp(' ')
disp('--- Testing critical point q = [pi/2,-1.2217,0,1.2217,pi/2] ---')
q_in = [pi/2,-1.2217,0,1.2217,pi/2];
w_in = dkf(q_in,l);
q_out = ikf(w_in,l);
disp(['Configuration in critical point: ' mat2str(q_out)])

disp(' ')
disp('--- Checking validity of direct kinematics ---');
load('./data/kinematics.mat')
err = zeros(size(q_in));
for i=1:size(q_in, 1)
  T_student = dkf(q_in{i}, l);
  err(i) = norm(T_out_dk{i}(1:3,4)-T_student(1:3,4));
  end
  
disp(['Max error in feasible workspace: ' mat2str(max(err))])
disp(['Error rate (all points): ' mat2str(sum(err>1e-10)) '/' mat2str(length(err))])



disp(' ')
disp('--- Checking validity of inverse kinematics ---');
err = zeros(size(q_in));
for i=1:size(q_in, 1)
  q_student = ikf(T_in_ik{i}, l);
  err(i) = norm(q_in{i}-q_student(1,:));
  for k = 1:size(q_student,1)
    err_k = norm(q_in{i}-q_student(k,:));
    if err_k < err(i)
      err(i) = err_k;
      end
    end
  end
  
disp(['Max error in feasible workspace: ' mat2str(max(err))])
disp(['Error rate (all points): ' mat2str(sum(err>1e-10)) '/' mat2str(length(err))])

disp(' ')
disp('--- Uniformly testing the feasible workspace for consistency---');
count = 1;
q = zeros(n^4,5);
err = zeros(n^4,1);
%insert l5 = 0 for consistency check
l(5)=0;

for q1 = linspace(q_min(1),q_max(1),n)
    for q2 = linspace(q_min(2),q_max(2),n)
        for q4 = linspace(q_min(4),q_max(4),n)
            for q5 = linspace(q_min(5),q_max(5),n)
                q_in = [q1;q2;0;q4;q5]';
                q(count,:) = q_in;
                w_in = dkf(q_in,l);
                q_out = ikf(w_in,l);
                err(count) = 1000;
                % Do check in w-space, to eliminate false errors
                % when the IK solution is underdetermined
                for k = 1:size(q_out,1)
                    w_out = dkf(q_out(k,:),l);
                    if isvector(w_out)
                        % IK output is vector, assuming it is
                        %[x;y;z;phi;theta;psi] with
                        % Cardan (Euler) angles in in z-y-x convention
                        err_k = norm(w_in(1:3)-w_out(1:3));%wrapToPi(w_in(4:6)-w_out(4:6))]);
                    else
                        % IK output is the tool transform matrix
                        err_k = norm(w_in(1:3,4)-w_out(1:3,4));%wrapToPi(w_in(4:6)-w_out(4:6))]);
                    end
                    if err_k < err(count)
                        err(count) = err_k;
                    end
                end
                % TODO: Add check in q-space
                count = count + 1;
            end
        end
    end
end

disp(['Max error in feasible workspace: ' mat2str(max(err))])
disp(['Error rate (all points): ' mat2str(sum(err>1e-10)) '/' mat2str(length(err))])

l = [15,105,55.95,57.50,12.31];

disp(' ')
disp('--- Checking initial conditions for direct kinematics ---')
T_0_0 = [-1 0 0 -15.00; 0 0 1 12.31; 0 1 0 -218.45; 0 0 0 1];

T_0_student_0 = dkf([0, -pi/2, 0, 0, 0], l);

if max(T_0_0 - T_0_student_0) < 1e-6
    disp('Looks like initial conditions are ok for direct kinematics.')
    disp('If you have errors in inverse kinematics, make sure to ')
    disp('return values that include inital conditions.')
else
    disp('Looks like you did not include initial conditions in direct kinematics')
end