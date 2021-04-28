clear all
clc

tic
M2 = readtable('LU_Network/NetworkConfig.csv');
Node = table2struct(M2);
toc

node_info = Node(3).IP