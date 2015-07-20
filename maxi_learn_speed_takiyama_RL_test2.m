function [] = maxi_learn_speed_takiyama_RL_test2()
% Reinforcement learning for force production of a two joint planar arm
% theta: direction of desired force (degrees)

%clear all; close all; clc;

% Input layer:
T = 100; %number of trial
K = 10; %number of position of theta in the circle (real number is K+1)
thetak = (2*pi)/K:(2*pi)/K:2*pi;
randomOrder = randi(K,T);
mOutput = 2; %number of outputs
nInput = 7; %number of neurons
phiInput = 30; % angle of perturbation

nTrials = 50;
phi = zeros(1,nInput);
v = zeros(1,nTrials);

% Connectivity (weight) matrix (W)
Z=zeros(mOutput, nInput);
%E=zeros(1,nTrials); %Faux, car E un reel
W = zeros(nInput, mOutput);
R = [cos(phiInput),-sin(phiInput);sin(phiInput),cos(phiInput)];
A = zeros(nInput,1);
%Lambda=nInput*R*Z*transpose(Z)*transpose(R); % realiser plus tard
phi(1)=rand;
Z(:,1) =(1/nInput)*[cos(phi(1)*2*pi),sin(phi(1)*2*pi)]';
D = zeros(mOutput, mOutput);
lambda= NaN(1,mOutput);
desMagnitude=1;
%xOutput = zeros(mOutput,1);
xOutput = [0,0]';
B=1;
sigmaA=0.05; 
epsilon=normrnd(0,sigmaA,nTrials,1);
sigmaP=0.05;
zeta=normrnd(0,sigmaP,nTrials,mOutput);
alpha=0.9;
randomOrder2 = randi([1,nInput],nTrials,1);

E = zeros(nTrials,1);

for k = 2:nInput
    phi(k)= rand;    % !!!!!Angle!!!this phi concerns about tge foce direction(FD)
    Y = (1/nInput)*[cos(phi(k)*2*pi);sin(phi(k)*2*pi)]';
    Z(:,k)=Y;
end

Lambda=nInput*R*Z*transpose(Z)*transpose(R)
%A = (R*Z)\xOutput; % essaie, a remettre dans l'algo

%for i = 1:mOutput
%   for j= 1:mOutput
%      if i==j
%     D(i,i)=lambda(i); %Autre element par default a zero?
%    else
%       D(i,j)= 0;
%  end
%end
%end

for j = 1:T
    desTheta = thetak(randomOrder(j));
    tTarget = desMagnitude*[cos(desTheta);sin(desTheta)] %t,target position
    %end
    %xOutput = R*Z*A; %Output that we obtain
    e = tTarget - xOutput; %error betweeen what we expect and what we have
    [V,D]=eig(Lambda);
    v= V*e ;
    % A = (R*Z)\xOutput;
    
    
    % for i = 1:nTrials-1
    
    
    %    %e(i+1)=(eye(mOutput)-B*Lambda)*e(i)
    
    %     %E(i+1)= (1/2)*transpose(e(i+1))*e(i+1);
    %     A= A + B*nInput*transpose(Z)*transpose(R)*transpose(V)*v;   %A(i+1)=A(i)+B*N*transpose(Z)*transpose(R)*transpose(V)*v(i);
    %    xOutput = R*Z*A;
    %     v=(eye(mOutput)-B*D)*v;     %v(i+1)=(eye(mOutput)-B*D)*v(i);
    %     E(i)= (1/2)*transpose(v)*v;    %  E(i+1)= (1/2)*transpose(v(i+1))*v(i+1);
    %     %v(i)=V*e(i)
    
    
    % end
    
    
    %plot(E)
    % hold on
    
    %  disp(E)
    
    
    
    for i = 1:nTrials-1
        
        
        %e(i+1)=(eye(mOutput)-B*Lambda)*e(i)
        
        %E(i+1)= (1/2)*transpose(e(i+1))*e(i+1);
        %A= A + B*nInput*transpose(Z)*transpose(R)*transpose(V)*v;   %A(i+1)=A(i)+B*N*transpose(Z)*transpose(R)*transpose(V)*v(i);
        
        W=alpha*W+B*nInput*transpose(Z)*transpose(R)*transpose(V)*v*transpose(tTarget)+sigmaP*zeta(i);
        A=W*tTarget+sigmaA*epsilon(randomOrder2(i)); %epsilon varie avec i
        
        
        xOutput = R*Z*A;
        v=(alpha*eye(mOutput)-B*D)*(v+(1-alpha)*inv(alpha*eye(mOutput)-B*D)*V*tTarget);     %v(i+1)=(eye(mOutput)-B*D)*v(i);
        E2(i)= (1/2)*transpose(v)*v;    %  E(i+1)= (1/2)*transpose(v(i+1))*v(i+1);
        %v(i)=V*e(i)
        
    end
    %plot(E2);
   %hold on
    
    plot(E2')
  
end



