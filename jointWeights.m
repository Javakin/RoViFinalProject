% Weights
clear all
close all 
clc


q0 = 425+392+94.75;
q1 = q0;
q2 = 392+94.75;
q3 = 94.75;
q4 = 82.5;
q5 = 10;

Q = [q0 q1 q2 q3 q4 q5]

Qnun = 1./Q
Qnun/sum(Qnun)
