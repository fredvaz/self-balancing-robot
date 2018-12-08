
function [ data ] = read(u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


global Serial sol

dat = uint8(fread(Serial, 2));

sol = swapbytes(typecast(uint8(dat), 'int16'));

data = double(sol);

end

