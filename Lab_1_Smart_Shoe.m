%% Lab 1: Smart Shoe
%Reading data
dataNormal = csvread('CollinNormal.csv');
trimDataNorm = dataNormal(250:end,:);
filteredDataNorm = weighted3filter(trimDataNorm);
dataInToe = csvread('CollinInToeing.csv');
trimDataInToe = dataInToe(250:end,:);
filteredDataInToe = weighted3filter(trimDataInToe);
dataHeel = csvread('JacobHeelWalking.csv');
trimDataHeel = dataHeel(250:end,:);
filteredDataHeel = weighted3filter(trimDataHeel);
dataOutToe = csvread('CollinOutToeing.csv');
trimDataOutToe = dataOutToe(250:end,:);
filteredDataOutToe = weighted3filter(trimDataOutToe);
dataTipToe = csvread('CollinTipToeing.csv');
trimDataTipToe = dataTipToe(250:end,:);
filteredDataTipToe = weighted3filter(trimDataTipToe);
%% step and lengths
[CDstep,CDstride] = stepLength(277);
[JWstep,JWstride] = stepLength(263.75);
[DTstep,DTstride] = stepLength(275.5);
[JGstep,JGstride] = stepLength(288.5);

%% cadence
%has indices of when step finished
[cadenceNorm, stepDataOnlyNorm] = findCadence(filteredDataNorm);
[cadenceHeel, stepDataOnlyHeel] = findCadenceHeel(filteredDataHeel);
[cadenceInToe, stepDataOnlyInToe] = findCadence(filteredDataInToe);
[cadenceOutToe, stepDataOnlyOutToe] = findCadence(filteredDataOutToe);
[cadenceTipToe, stepDataOnlyTipToe] = findCadenceTipToe(filteredDataTipToe);
%% calculating MFP
MFPnorm = findMFP(stepDataOnlyNorm);
MFPHeel = findMFP(stepDataOnlyHeel);
MFPInToe = findMFP(stepDataOnlyInToe);
MFPOutToe = findMFP(stepDataOnlyOutToe);
MFPTipToe = findMFP(stepDataOnlyTipToe);
%% speed
%speed outputted is in inches/min
speedNorm = findSpeed(CDstep,cadenceNorm);
speedHeel = findSpeed(CDstep,cadenceHeel);
speedTipToe = findSpeed(CDstep,cadenceTipToe);
speedInToe = findSpeed(CDstep,cadenceInToe);
speedOutToe = findSpeed(CDstep,cadenceOutToe);

%% printing important values
fprintf('Normal values \nCadence: %d strides/min  Speed: %d inches/min  MFP: %d \n',cadenceNorm,speedNorm,MFPnorm);
fprintf('In Toe values \nCadence: %d strides/min  Speed: %d inches/min  MFP: %d \n',cadenceInToe,speedInToe,MFPInToe);
fprintf('Out Toe values \nCadence: %d strides/min  Speed: %d inches/min  MFP: %d \n',cadenceOutToe,speedOutToe,MFPOutToe);
fprintf('Heel Walk values \nCadence: %d strides/min  Speed: %d inches/min  MFP: %d \n',cadenceHeel,speedHeel,MFPHeel);
fprintf('Tip Toe values \nCadence: %d strides/min  Speed: %d inches/min  MFP: %d \n',cadenceTipToe,speedTipToe,MFPTipToe);

%% functions
% step length function
function [step,stride] = stepLength(x)
%cm/step, input in inches, second conversion, 10 steps done
step = (2.54*x)/10;
stride = 2*step;
end

% finding cadence and stepping data function
function [cadence,stepDataOnly] = findCadence(pressureMat)
% this function finds the cadence of a persons walking profile based on 
% data from the pressure sensors
% It also isolates the data from when someone is stepping on the foot with 
% the sensor, which will be used for calculating MFP later on

MF = pressureMat(:,3); %takes MF data
heel = pressureMat(:,4); %takes heel data
MM = pressureMat(:,1); %takes MM data
LF = pressureMat(:,2); %takes LF data
nSamples = length(MF); 
fs = 30; %sampling frequency

%finding how many steps in the trial
strideInd = [];
for i = 1:nSamples
    if i+1 > nSamples 
        break
    end
    if MF(i) > 200 && MF(i+1) < 200
        strideInd = [strideInd,i]; %takes the end of each stride, last value
    end
end

%finding the start and stop of each step
stepStart = [];
stepEnd = [];
for i = 1:nSamples
    if i+1 > nSamples
        break
    end
    if heel(i) < 200 && heel(i+1) > 200
        stepStart = [stepStart,i+1]; %takes the beginning of heel strike
    end
    if MF(i) > 200 && MF(i+1) < 200
        stepEnd = [stepEnd,i]; %takes end of MM coming off ground 
    end
end

%grabbing only step data
newMM = [];
newMF = [];
newLF = [];
newHeel = [];
for j = 1:length(stepStart)
    %if statement incase stepEnd/Start arent the same size
    if j+1 > length(stepStart) || j+1 > length(stepEnd)
        break
    end
    %EACH INDIVIDUAL STEP DATA IS BROKEN UP WITH BY A -1, HENCE THE -1
    %ADDED AT THE END OF EACH ITERATION
    newMM = [newMM;MM(stepStart(j):stepEnd(j));-1];
    newMF = [newMF;MF(stepStart(j):stepEnd(j));-1];
    newLF = [newLF;LF(stepStart(j):stepEnd(j));-1];
    newHeel = [newHeel;heel(stepStart(j):stepEnd(j));-1];
end

%putting the new data into a matrix
stepDataOnly = [newMM, newLF, newMF, newHeel];

%finding stride time
halfInd = ceil(length(strideInd)/2);
numStride = length(strideInd);
numStep = numStride / 2;
strideTime = (strideInd(halfInd+1) - strideInd(halfInd)) / fs;
stepTime = strideTime / 2;

%finding cadence
stepInMin = 60*numStep/stepTime;
cadence = stepInMin / 60;
end

% 3 point weighted average filter function
function w3 = weighted3filter(data)
for j = 1:length(data(1,:))
    for i = 2: length(data)-1
        w3(i-1,j) = (data(i-1,j) + 2*data(i,j) + data(i+1,j)) / 4;
    end
end
end

% function to calculate MFP
function MFP = findMFP(dataMat)
% this function calculates the MFP based on the average value of the 4
% sensors during each step
% the dataMat entered needs to be the output variable stepDataOnly from the
% findCadence function 
MMvec = dataMat(:,1);
LFvec = dataMat(:,2);
MFvec = dataMat(:,3);
heelVec = dataMat(:,4);

%finding where each step starts and ends
stepInd = find(MMvec == -1);

% taking only one step and using those average values to calculate MFP
halfInd = ceil(length(stepInd)/2); %takes the middle step, gives good data 
newMM = MMvec(stepInd(halfInd):stepInd(halfInd+1));
newMF = MFvec(stepInd(halfInd):stepInd(halfInd+1));
newLF = LFvec(stepInd(halfInd):stepInd(halfInd+1));
newHeel = heelVec(stepInd(halfInd):stepInd(halfInd+1));

% averaging the pressure sensor values to calculate MFP
avgMM = mean(newMM);
avgLF = mean(newLF);
avgMF = mean(newMF);
avgHeel = mean(newHeel);

% calculating MFP
MFP = ((avgMM+avgMF) *100) / (avgMM + avgMF + avgLF + avgHeel + .001);
end

%speed function
function speed = findSpeed(stepLength,cadence)
speed = stepLength*cadence;
end

function [cadence,stepDataOnly] = findCadenceHeel(pressureMat)
% this function finds the cadence of a persons walking profile based on 
% data from the pressure sensors
% It also isolates the data from when someone is stepping on the foot with 
% the sensor, which will be used for calculating MFP later on

MF = pressureMat(:,3); %takes MF data
heel = pressureMat(:,4); %takes heel data
MM = pressureMat(:,1); %takes MM data
LF = pressureMat(:,2); %takes LF data
nSamples = length(MF); 
fs = 30; %sampling frequency

%finding how many steps in the trial
strideInd = [];
for i = 1:nSamples
    if i+1 > nSamples 
        break
    end
    if heel(i) > 400 && heel(i+1) < 400
        strideInd = [strideInd,i]; %takes the end of each stride, last value
    end
end

%finding the start and stop of each step
stepStart = [];
stepEnd = [];
for i = 1:nSamples
    if i+1 > nSamples
        break
    end
    if heel(i) < 400 && heel(i+1) > 400
        stepStart = [stepStart,i+1]; %takes the beginning of heel strike
    end
    if heel(i) > 400 && heel(i+1) < 400
        stepEnd = [stepEnd,i]; %takes end of MM coming off ground 
    end
end

%grabbing only step data
newMM = [];
newMF = [];
newLF = [];
newHeel = [];
for j = 1:length(stepStart)
    %if statement incase stepEnd/Start arent the same size
    if j+1 > length(stepStart) || j+1 > length(stepEnd)
        break
    end
    %EACH INDIVIDUAL STEP DATA IS BROKEN UP WITH BY A -1, HENCE THE -1
    %ADDED AT THE END OF EACH ITERATION
    newMM = [newMM;MM(stepStart(j):stepEnd(j));-1];
    newMF = [newMF;MF(stepStart(j):stepEnd(j));-1];
    newLF = [newLF;LF(stepStart(j):stepEnd(j));-1];
    newHeel = [newHeel;heel(stepStart(j):stepEnd(j));-1];
end

%putting the new data into a matrix
stepDataOnly = [newMM, newLF, newMF, newHeel];

%finding stride time
halfInd = ceil(length(strideInd)/2);
numStride = length(strideInd);
numStep = numStride / 2;
strideTime = (strideInd(halfInd+1) - strideInd(halfInd)) / fs;
stepTime = strideTime / 2;

%finding cadence
stepInMin = 60*numStep/stepTime;
cadence = stepInMin / 60;
end

function [cadence,stepDataOnly] = findCadenceTipToe(pressureMat)
% this function finds the cadence of a persons walking profile based on 
% data from the pressure sensors
% It also isolates the data from when someone is stepping on the foot with 
% the sensor, which will be used for calculating MFP later on

MF = pressureMat(:,3); %takes MF data
heel = pressureMat(:,4); %takes heel data
MM = pressureMat(:,1); %takes MM data
LF = pressureMat(:,2); %takes LF data
nSamples = length(MF); 
fs = 30; %sampling frequency

%finding how many steps in the trial
strideInd = [];
for i = 1:nSamples
    if i+1 > nSamples 
        break
    end
    if LF(i) > 200 && LF(i+1) < 200
        strideInd = [strideInd,i]; %takes the end of each stride, last value
    end
end

%finding the start and stop of each step
stepStart = [];
stepEnd = [];
for i = 1:nSamples
    if i+1 > nSamples
        break
    end
    if LF(i) < 200 && LF(i+1) > 200
        stepStart = [stepStart,i+1]; %takes the beginning of heel strike
    end
    if LF(i) > 200 && LF(i+1) < 200
        stepEnd = [stepEnd,i]; %takes end of MM coming off ground 
    end
end

%grabbing only step data
newMM = [];
newMF = [];
newLF = [];
newHeel = [];
for j = 1:length(stepStart)
    %if statement incase stepEnd/Start arent the same size
    if j+1 > length(stepStart) || j+1 > length(stepEnd)
        break
    end
    %EACH INDIVIDUAL STEP DATA IS BROKEN UP WITH BY A -1, HENCE THE -1
    %ADDED AT THE END OF EACH ITERATION
    newMM = [newMM;MM(stepStart(j):stepEnd(j));-1];
    newMF = [newMF;MF(stepStart(j):stepEnd(j));-1];
    newLF = [newLF;LF(stepStart(j):stepEnd(j));-1];
    newHeel = [newHeel;heel(stepStart(j):stepEnd(j));-1];
end

%putting the new data into a matrix
stepDataOnly = [newMM, newLF, newMF, newHeel];

%finding stride time
halfInd = ceil(length(strideInd)/2);
numStride = length(strideInd);
numStep = numStride / 2;
strideTime = (strideInd(halfInd+1) - strideInd(halfInd)) / fs;
stepTime = strideTime / 2;

%finding cadence
stepInMin = 60*numStep/stepTime;
cadence = stepInMin / 60;
end