function [] = maxi_learn_speed_takiyama_RL2_2()
% Reinforcement learning for force production of a two joint planar arm
% theta: direction of desired force (degrees)

clear all; close all; clc;

% Input layer:
T = 1; %number of trial
K = 10; %number of position of theta in the circle (real number is K+1)
thetak = (2*pi)/K:(2*pi)/K:2*pi;
randomOrder = randi(K,T);
mOutput = 2; %number of outputs
nInput = 3; %number of neurons
phiInput = 30; % angle of perturbation

nTrials = 20000;
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
expectR1= zeros(nInput,1);
O1= zeros(nInput,1);
A1= zeros(nInput,1);
y= zeros(nInput,1);
omega1= zeros(nInput,1);
Wthres2 = zeros(nInput,1);
Vthres2= zeros(nInput,1);
deltaW1= zeros(nInput,1);
deltaV1= zeros(nInput,1);
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
r1=zeros(nInput,1);
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
        for p = 1:nInput
            expectr=expectR1(p);
            O1=[(1-expectr)/2 0];
            omega1(p)=max(O1);
            A1(p)=normrnd(mu1(p),omega1(p));
            y(p)=(2./(1+ exp(-A1(p))))-1;
            
            
            %      mu2=W1*tTarget+Wthres2;   %first element of Y2
            %    expectR2=V1*tTarget+Vthres2;
            %     O2=[(1-expectR2)/2 0];
            %     omega2=max(O2);
            %     A2=normrnd(mu2,omega2);
            %     y(2)=(2./(1+ exp(-A2)))-1;
        end
        %determination of X the output
        xOutput=Z*y*(1/nInput);
        % xOutput(1)=U1(1);
        % U2=Z*y(2);
        % xOutput(2)=U2(2);
        %RL for the first element of the output
        e = (1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
        
        %RL for the second element of the output
        % e2 = (1/2)*((tTarget(2))-Z(2)*y(2))^2;
        r= max(0,(rewardThreshold - e/(2^nInput))/rewardThreshold);
        
        r1 = r*ones(nInput,1);
        for p = 1:nInput
            for k = 1:mOutput
                deltaW1= (r1-expectR1).*((A1-mu1)./omega1);  %RL W
                W1(p,k)=W1(p,k)+alpha1*deltaW1(p)*tTarget(k);
                Wthres1(p)=Wthres1(p)+alpha1*deltaW1(p);
                
                deltaV1 = (r1-expectR1); %RL V
                V1(p,k)=V1(p,k)+beta1*deltaV1(p)*tTarget(k);
                
            end
            
        end
        
        
        EG(i)=(1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
        plot(EG)
    end
    % EG(i)=(1/2)*(tTarget-xOutput)'*(tTarget-xOutput);
    
end

