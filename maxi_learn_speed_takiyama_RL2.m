function [] = maxi_learn_speed_takiyama_RL2()
% Reinforcement learning for force production of a two joint planar arm
% theta: direction of desired force (degrees)

%clear all; close all; clc;

% Input layer:
T = 1; %number of trial
K = 10; %number of position of theta in the circle (real number is K+1)
thetak = (2*pi)/K:(2*pi)/K:2*pi;
randomOrder = randi(K,T);
mOutput = 2; %number of outputs
nInput = 1; %number of neurons
phiInput = 30; % angle of perturbation

nTrials = 50000;
phi = zeros(1,nInput);
v = zeros(1,nTrials);

% Connectivity (weight) matrix (W)
Z=zeros(mOutput, nInput);
%E=zeros(1,nTrials); %Faux, car E un reel
W1 = rand(nInput, mOutput);
W2= rand(nInput, mOutput);
V1 = rand(nInput, mOutput);
V2= rand(nInput, mOutput);
Wthres1 = zeros(nInput,1);
Vthres1= zeros(nInput,1);

Wthres2 = zeros(nInput,1);
Vthres2= zeros(nInput,1);
%R = [cos(phiInput),-sin(phiInput);sin(phiInput),cos(phiInput)];
A = zeros(nInput,1);
%Lambda=nInput*R*Z*transpose(Z)*transpose(R); % realiser plus tard
phi(1)=rand;
Z(:,1) =(1/nInput)*[cos(phi(1)*2*pi),sin(phi(1)*2*pi)]';
D = zeros(mOutput, mOutput);
lambda= NaN(1,mOutput);
desMagnitude=1;
%xOutput = zeros(mOutput,1);
xOutput = [0,0]';
U1 = [0,0]';
U2 = [0,0]';
B=1;
rewardThreshold=1;
alpha1=1/4;
beta1=1/2;
alpha2=1/4;
beta2=1/2;
E = zeros(nTrials,1);

%for k = 2:nInput
%   phi(k)= rand;    % !!!!!Angle!!!this phi concerns about tge foce direction(FD)
%  Y = (1/nInput)*[cos(phi(k)*2*pi);sin(phi(k)*2*pi)]';
% Z(:,k)=Y;
%end


for j = 1:T
    desTheta = thetak(randomOrder(j));
    tTarget = desMagnitude*[cos(desTheta);sin(desTheta)] %t,target position
    
    
    for i = 1:nTrials
        mu1=W1*tTarget+Wthres1; %first element of Y1
        expectR1=V1*tTarget+Vthres1;
        O1=[(1-expectR1)/2 0];
        omega1=max(O1);
        A1=normrnd(mu1,omega1);
        y(1)=1./(1+ exp(-A1));
        
        
        mu2=W1*tTarget+Wthres2;   %first element of Y2
        expectR2=V1*tTarget+Vthres2;
        O2=[(1-expectR2)/2 0];
        omega2=max(O2);
        A2=normrnd(mu2,omega2);
        y(2)=1./(1+ exp(-A2));
        
        %determination of X the output
        U1=Z*y(1);
        xOutput(1)=U1(1);
        U2=Z*y(2);
        xOutput(2)=U2(2);
        %RL for the first element of the output
        e1 = (1/2)*((tTarget(1))-Z(1)*y(1))^2;
        r1= max(0,(abs(rewardThreshold - e1))/rewardThreshold);
        
        %RL for the second element of the output
        e2 = (1/2)*((tTarget(2))-Z(2)*y(2))^2;
        r2= max(0,(abs(rewardThreshold - e2))/rewardThreshold);
        
        for k = 1:mOutput
            deltaW1= (r1-expectR1)*((A1-mu1)/omega1);  %RL W
            W1(k)=W1(k)+alpha1*deltaW1*tTarget(k);
            Wthres1=Wthres1+alpha1*deltaW1;
            
            deltaV1 = (r1-expectR1); %RL V
            V1(k)=V1(k)+beta1*deltaV1*tTarget(k);
            
           
            
        end
        
        for k = 1:mOutput
            deltaW2= (r2-expectR2)*((A2-mu2)/omega2);  %RL W
            W2(k)=W2(k)+alpha2*deltaW2*tTarget(k);
            Wthres2=Wthres2+alpha2*deltaW2;
            
            deltaV2 = (r2-expectR2); %RL V
            V2(k)=V2(k)+beta2*deltaV2*tTarget(k);
        end
        EG(i)=(1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
        plot(EG)
    end
    % EG(i)=(1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
    
end

