function[kl,c,rcor]=truss_kl(i,prop)
global geom connec
Ei=prop(i,1);
Ai=prop(i,2);
x1=geom(connec(i,1),1);
y1=geom(connec(i,1),2);
x2=geom(connec(i,2),1);
y2=geom(connec(i,2),2);
l=sqrt((x2-x1)^2+(y2-y1)^2);
kl=[Ei*Ai/l 0 -Ei*Ai/l 0;
    0 0 0 0;
    -Ei*Ai/l 0 Ei*Ai/l 0;
    0 0 0 0];
if (x2-x1)==0
     theta=((y2-y1)/abs(y2-y1))*2*atan(1);
else
    theta=atan((y2-y1)/(x2-x1));
end
rcor=[cos(theta) sin(theta);-sin(theta) cos(theta)]*[(x2-x1);(y2-y1)];
c=[cos(theta) -sin(theta) 0 0;
    sin(theta) cos(theta) 0 0;
    0 0 cos(theta) -sin(theta);
    0 0 sin(theta) cos(theta)];
end 

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
