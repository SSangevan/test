function [] = maxi_learn_speed_takiyama_RL()
% Reinforcement learning for force production of a two joint planar arm
% theta: direction of desired force (degrees)

%clear all; close all; clc;

% Input layer:
T = 100; %number of trial
K = 5; %number of position of theta in the circle (real number is K+1)
thetak = (2*pi)/K:(2*pi)/K:2*pi;
randomOrder = randi(K,T);
mOutput = 2; %number of outputs
nInput = 10; %number of neurons
phiInput = 0; % angle of perturbation
rewardThreshold = 1;
Wthres = zeros(nInput,1);
Vthres= zeros(nInput,1);

nTrials = 10;
phi = zeros(1,nInput);
%v = zeros(1,nTrials);

% Connectivity (weight) matrix (W)
Z=zeros(mOutput, nInput);

%E=zeros(1,nTrials); %Faux, car E un reel
%W = zeros(nInput, mOutput);
W = rand(nInput, mOutput);
V = zeros(nInput, mOutput);
R = [cos(phiInput),-sin(phiInput);sin(phiInput),cos(phiInput)];
A = zeros(nInput,1);
y = zeros(nInput,1);
%Lambda=nInput*R*Z*transpose(Z)*transpose(R); % realiser plus tard
phi(1)=rand;
Z(:,1) =(1/nInput)*[cos(phi(1)*2*pi),sin(phi(1)*2*pi)]';
r=0;
alpha1=1/2;
beta1=1/2;

desMagnitude=1;
%xOutput = zeros(mOutput,1);
xOutput = [0,0]';
B=1;
%sigmaA=0.05;
%epsilon=normrnd(0,sigmaA,nInput,1);
%sigmaP=0.05;
%zeta=normrnd(0,sigmaP,nInput,mOutput);
%alpha=0.9;
%randomOrder2 = randi([1,nInput],nTrials,1);

%E = zeros(nTrials,1);
%Reward=zeros(T,1);
E=zeros(T,1);

for k = 2:nInput
    phi(k)= rand;    % !!!!!Angle!!!this phi concerns about tge foce direction(FD)
    Y = (1/nInput)*[cos(phi(k)*2*pi),sin(phi(k)*2*pi)]';
    Z(:,k)=Y;
   
end


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
    
    for i = 1:nInput
        mu(i)=W(i,:)*tTarget+Wthres(i);
        expectR(i)=V(i,:)*tTarget+Vthres(i);
        O=[(1-expectR(i))/2 0];
        omega(i)=max(O);
        A(i)=normrnd(mu(i),omega(i));
        y(i)=1./(1+ exp(-A(i)));
        
    end
    
    xOutput=(1/nInput)*R*Z*y
    cost=(1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
    r= max(0,(rewardThreshold - (cost))/rewardThreshold);
    %E(i)=cost;
    
    if xOutput==[NaN NaN]';
        then display(j);
    end
        
        
        %Reinforcement Learning
        
        for i = 1:nInput
            deltaW(i) = (r-expectR(i))*((A(i)-mu(i))/omega(i));
            deltaV(i) = (r-expectR(i));
            
            for k = 1:mOutput
                W(i,k) = W(i,k) + alpha1*deltaW(i)*tTarget(k);
                Wthres(i) = Wthres(i) + alpha1*deltaW(i);
                V(i,k) = V(i,k) + beta1*deltaV(i)*tTarget(k);
                
                
            end
        end
        
        X(:,j)=xOutput;
        E(j)=cost;
        
    end
    %xOutput=R*Z*A
    %cost=(1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
    %r = max(0,(rewardThreshold - cost)/rewardThreshold);
    %Q(j)=r;
    %E(j)=cost;
    
    
    
    
    
    
    
    
    plot(E)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
