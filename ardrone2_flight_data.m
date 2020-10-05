function [t,ts,u,x,y,A,B,C] = ardrone2_flight_data
load ardrone2FlightData.mat t ts u x y A B C;
u = u;
x = x';
y = y';
end