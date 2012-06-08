function [Bk,Knots]=mysplineVer1(t,d,m),
  %% [Bk,,Knots]=myspline(t,d,m),this function calculs spline
  %% functions. t contains the time sampling, d is the degree of Bspline 
%%functions and m is the number of the functions
 
  Bk=[];

Knots=[1:fix(length(t)/(m+d+2)):length(t)];

for i=1:length(Knots)-1,
	Bk1=zeros(1,length(t));
        Bk1(1,Knots(i):Knots(i+1)-1)=ones(1,Knots(i+1)-Knots(i));
        Bk=[Bk;Bk1];
end;

disp(Bk)

for j=1:d,
	Bkkk=[];
        [ln,cn]=size(Bk);
        for k=1:ln-1,
                Bkd= (t-t(Knots(k)))/(t(Knots(k+j))-t(Knots(k))).*Bk(k,:)+...
		(t(Knots(k+1+j))-t)/(t(Knots(k+j+1))-t(Knots(k+1))).*Bk(k+1,:);
                Bkkk=[Bkkk;Bkd];
       end;
       Bk=Bkkk;
end;

Bk=Bk';
return
