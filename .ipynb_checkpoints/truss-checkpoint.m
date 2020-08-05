E=30000;
A1=45000;
A2=20000;
global nnd nel nne nodof eldof geom connec prop nf load
nnd = 9; % Number of nodes:
nel = 15; % Number of elements:
nne = 2 ; % Number of nodes per element:
nodof =2 ; % Number of degrees of freedom per node
eldof = nne*nodof; % Number of degrees of freedom
% Nodes coordinates X and Y
geom=zeros(nnd,2);
geom=[0 0;
    2 0;
    4 0;
    6 0;
    8 0;
    7 2;
    5 2;
    3 2;
    1 2];
connec=zeros(nel,2);
connec=[1 2;
    2 3;
    3 4;
    4 5;
    5 6;
    6 7;
    7 8;
    8 9;
    9 1;
    2 9;
    2 8;
    3 8;
    7 3;
    4 7;
    6 4];
full=[1 2 3 4 6 7 8];
prop=zeros(nel,2);
for i=1:nel
     prop(i,:)=[E A2];
    for j=1:7
        if i==full(j)
            prop(i,:)=[E A1];
        end
    end
end
nf = ones(nnd, nodof); % Initialize the matrix nf to 1
nf(1,:) =[0 0];
nf(5,:) =[1 0] ; % Prescribed nodal freedom of node 5
 
%
n=0;
for i=1:nnd
for j=1:nodof
if nf(i,j) ~= 0
n=n+1;
nf(i,j)=n;
end
end
end
load=[0 0;
    0 -5000;
    0 0;
    0 -10000;
    0 0;
    0 0;
   0 0;
   0 -7000;
   15000 0];

%global force matrix
fg=zeros(n,1);
for i=1:nnd
    for j=1:nodof
        if(nf(i,j))~=0;
            fg(nf(i,j),1)=fg(nf(i,j),1)+load(i,j);
        end
    end
end



%global assembled stiffness matrix
kk=zeros(n,n);
rcord=zeros(nel,2);
for i=1:nel
    [kl,c,rcor]=truss_kl(i,prop);
    kg=c*kl*c';
    g=truss_g(i);
    kk=form_kk(kk,kg,g);
    rcord(i,:)=rcor;
end
delta=kk\fg;
displacement=zeros(nnd,nodof);
for i=1:nnd
    for j=1:nodof
        if nf(i,j)~=0
            displacement(i,j)=delta(nf(i,j),1);
        end
    end
end
%dislacement at nodes of element in global coordinate system
deltaeg=zeros(nel,4);
deltael=zeros(nel,4);
forcelocale=zeros(nel,4);
for i=1:nel
    deltaeg(i,:)=[displacement(connec(i,1),1) displacement(connec(i,1),2) displacement(connec(i,2),1) displacement(connec(i,2),2)];
    [kl,c,rcor]=truss_kl(i,prop);
    deltael(i,:)=c'*deltaeg(i,:)';
    forcelocale(i,:)=kl*deltael(i,:)';
    
end
factor=1000;
x=zeros(nne,nel);
y=zeros(nne,nel);
X=zeros(nne,nel);
Y=zeros(nne,nel);
xy=cell(1,nel);
XY=cell(1,nel);
fm=[1 2];
profile=zeros(nne,nel);
Profile=cell(1,nel);
for i=1:nel
    for j=1:nne
        x(j,i)=geom(connec(i,j),1);
        y(j,i)=geom(connec(i,j),2);
        X(j,i)=geom(connec(i,j),1)+factor*displacement(connec(i,j),1);
        Y(j,i)=geom(connec(i,j),2)+factor*displacement(connec(i,j),2);
        profile(j,i)=displacement(connec(i,j),1);
    end
    xy{i}=[x(:,i) y(:,i)];
    XY{i}=[X(:,i) Y(:,i)];
    Profile{i}=[profile(:,i)];
end
figure
        set(gcf,'color','w')
        
cellfun(@patch,repmat({'Vertices'},1,nel),xy,.......
            repmat({'Faces'},1,nel),repmat({fm},1,nel),......
            repmat({'FaceVertexCdata'},1,nel),Profile,......
            repmat({'Edgecolor'},1,nel),repmat({'interp'},1,nel)); 
% cellfun(@patch,repmat({'Vertices'},1,nel),XY,.......
%            repmat({'Faces'},1,nel),repmat({fm},1,nel),......
%            repmat({'Edgecolor'},1,nel),repmat({'green'},1,nel)); 
%        view(2)
%        set(gca,'XTick',[]) ; set(gca,'YTick',[]);

axis auto
        
    
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
