%   GENERATE MPC PARAMETERS FROM MODEL
%   
%       JK 3/7/13
%
%   REQUIRED:
%       - FUNCTION 'printCMatrix.m' FOR MATRIX PRINTING
%       - IDDATA OBJECT 'mpd' WITH SYSTEM MODEL 
%

clear all;

% load model data into SS object
[G,H,C,D] = ssdata(mpd); % = ssdata(mp);
Ts = 3.333333333e-3; % sampling time in sec

n = size(G,1); % number of states
m = size(H,2); % number of inputs
p = size(C,1); % number of outputs

sysd = ss(G,H,C,D,Ts);
sysdtot = ss(G,H,eye(n),zeros(n,m),Ts);

% MPC parameters
Hp = 5;   % prediction horizon

% Q and R weighting matrices defined as lines along diagonal
%   specify line parameters
qi1=0.5; qf1=0.6; m1=(qf1-qi1)/Hp; % initial/final Q value, slope on error 1
qi2=0.8; qf2=0.7; m2=(qf2-qi2)/Hp; % initial/final Q value, slope on error 2
ri1=1; rf1=1; mr1=(rf1-ri1)/Hp; % initial/final R value, slope on input 1
ri2=1; rf2=1; mr2=(rf2-ri2)/Hp; % initial/final R value, slope on input 2

% initialize Q and R, create diagonal row, then make matrix
Q = [qi1 qi2]; % output error penalty
R = [ri1 ri2]; % control input penalty
for i=2:Hp,
    Q = [Q [qi1+m1*i qi2+m2*i]];
    R = [R [ri1+mr1*i ri2+mr2*i]];
end;
Q = diag(Q);
R = diag(R);

% calculate Kca
Kca = C*G;
for i = 2:Hp,
    Kca = [Kca; C*G^i];
end;

% calculate Kcab
dim = size(C*H);
for i = 1:Hp, % for each row
    for j = 1:Hp, % for each column
        % build first column element of row
        if j == 1,
            temp = C*G^(i-j)*H;
            continue;
        end;
        % fill in rest of columns
        if i >= j,
            temp = [temp C*G^(i-j)*H];
        else
            temp = [temp zeros(dim)];
        end;
    end;
    % add row
    if i == 1,
        Kcab = temp;
    else
        Kcab = [Kcab; temp];
    end;
end;

% calculate K
A = diag(1*ones(1,p*Hp)) + diag(-1*ones(1,p*Hp-1),-1); % for DeltaU instead of U
K = inv(Kcab'*Q*Kcab+R)*Kcab'*Q;
K1 = K(1:m,:); % take m rows

%
% create kalman estimator gain, L
Qn = 1e-2*eye(2);
% measurement covariance matrix, Rn, taken from experimental data
Rn = 1e-3*diag([.1413 0.1413]);
[kest,L,P] = kalman(sysd,Qn,Rn);

% make new combined system
Gbar = [G -H*K1*Kca; L*C G-H*K1*Kca-L*C];
Hbar = [H*K1; H*K1];
Cbar = [C zeros(p,n)];
sysdbar = ss(Gbar,Hbar,Cbar,zeros(p,Hp*p),Ts);

%
% test output to some input
tspan = 0:Ts:5;
N = length(tspan);

yd1 = 1*sin(2*pi*0.8*tspan) - cos(2*pi*0.7*tspan) + 2;
yd2 = 1*sin(2*pi*0.6*tspan) - cos(2*pi*0.5*tspan) + 5;

% make additional "ghost" values at end for prediction
yd = [yd1 yd1(N)*ones(1,Hp); 
      yd2 yd2(N)*ones(1,Hp)];

% build input vector, column by column
for j = 1:N, % for each column
    for i = 1:Hp, % for each row
        if i == 1,
            temp = yd(:,j+i);
        else
            temp = [temp; yd(:,j+i)];
        end;
    end;
    % add column
    if j == 1,
        Yd = temp;
    else
        Yd = [Yd temp];
    end;
end;
%             
[Y,T,X] = lsim(sysdbar,Yd,tspan);

% variable for simulink simulation
ts = timeseries(Yd,tspan);
% create separate time series with just yd, not Yd
tsyd = timeseries(Yd(1:p,:),tspan);
 
% % plot results
stairs(T,[yd1' yd2' Y]);
legend('yd1','yd2','y1','y2');

% Oputput relative matrices
fprintf(1, '\n\n*** Matrices in C form: ***\n\n');
fprintf(1, ['float G[n][n] =' printCMatrix(G) '\n']);
fprintf(1, ['float H[n][m] =' printCMatrix(H) '\n']);
fprintf(1, ['float C[p][n] =' printCMatrix(C) '\n']);
fprintf(1, ['float L[n][p] =' printCMatrix(L) '\n']);
fprintf(1, '\n');
fprintf(1, ['float Kca[Hp*p][n] =' printCMatrix(Kca) '\n']);
fprintf(1, ['float K1[m][Hp*p] =' printCMatrix(K1) '\n\n']);

