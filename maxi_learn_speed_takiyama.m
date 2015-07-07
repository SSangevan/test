function [] = maxi_learn_speed_takiyama()
% Reinforcement learning for force production of a two joint planar arm
% theta: direction of desired force (degrees)

%clear all; close all; clc;

% Input layer:
T = 15; %number of trial
K = 10; %number of position of theta in the circle (real number is K+1)
thetak = (2*180)/K:(2*180)/K:360;
randomOrder = randperm(T);
mOutput = 2; %number of outputs
nInput = 15; %number of neurons
phiInput = 30; % angle of perturbation
nTrials = 3000;
phi = zeros(1,nInput);
v = zeros(1,nTrials);

% Connectivity (weight) matrix (W)
Z=cell(mOutput, nInput);
E=cell(1,nTrials);
%W = zeros(nInput, mOutput);
R = [cos(phiInput),-sin(phiInput);sin(phiInput),cos(phiInput)];
A = zeros(nInput,1);
%Lambda=nInput*R*Z*transpose(Z)*transpose(R);
phi(1)=rand;
Z(:,1) =(1/nInput)*[cos(phi(1)),sin(phi(1))]';
D = zeros(mOutput, mOutput);



for k = 2:nInput
    phi(k)= rand;    % !!!!!Angle!!!this phi concerns about tge foce direction(FD)
    Y = [cos(phi(k));sin(phi(k))]';
    Z(:,k)=Y;
end
    
for i = 1:mOutput
    for j= 1:mOutput
        if i==j
        D(i,i)=lamda(i); %Autre element par default a zero?
        else
            D(i,j)= 0;
        end
    end
end   
    
for j = 1:T
    desTheta = thetak(randomOrder(j));
    desiredForce = desMagnitude*[cos(desTheta);sin(desTheta)]';%t,target position
    
    xOutput = R*Z*A; %Output that we obtain
    e = desiredForce - xOutput; %error betweeen what we expect and what we have
    [V,D]=eig(Lambda);
    
    for i = 1:nTrials-1
     v(1)= V*e ;  
     %e(i+1)=(eye(mOutput)-B*Lambda)*e(i)   
     
    %E(i+1)= (1/2)*transpose(e(i+1))*e(i+1);  
     A(i+1)=A(i)+B*N*transpose(Z)*transpose(R)*transpose(V)*v(i);
     E(i+1)= (1/2)*transpose(v(i+1))*v(i+1);  
     %v(i)=V*e(i)
     v(i+1)=(eye(mOutput)-D)*v(i);
    end
end


    
    

        
        