clc;clear all;
% pos1_modified =  [0.001,-0.00034,0.00015,-0.00037,0,-0.00019,-0.00054,-0.00024,-0.00015,-0.0001,-0.00017,-0.00014,-0.0001,-0.0001,-0.00012,-0.0001,-0.00015,-0.0002,-0.0002,-0.00019,-0.00012,-0.00016];
% pos2_modified = [-0.0161,-0.00899,-0.007,-0.005,-0.004,-0.0032,-0.0028,-0.00259,-0.00239,-0.002314,-0.0022,-0.0019,-0.0018,-0.0016,-0.0015,-0.0015,-0.0014,-0.0014,-0.0013,-0.0012,-0.00117,-0.00108];
% position_modified = pos1_modified-pos2_modified;
% force = ones(1,22)*1.96;%convert to Newton
% stiff_modified(:)=(force(:)./position_modified(:));
% for i=1:32
%    k(1,i)=i*100;
% end
% 
% 
%  pos1_unmodified=[0.0063, 0.0016 , 0.004 ,0.0010, 0.0009, 0.0006 ,0, 0.0001, -0.0001,0.0001, -0.0001,0.0001,0.0001, -0.0001, -0.0002, -0.0002, -0.0002, -0.0002, -0.0001, -0.0001, -0.0001, -0.0001, -0.0002, -0.0001, -0.0002, -0.0001,0,0, -0.0001, -0.0001, -0.0001,  -0.0001];
% % %p2 is the mean for lower values in position z data
% pos2_unmodified=[-0.0323,-0.0145,-0.0092,-0.007,-0.0058,-0.0052,-0.0036,-0.0032,-0.0028,-0.0027,-0.0024,-0.0023,-0.0020,-0.0019,-0.0018,-0.0016,-0.0016,-0.0017,-0.0015, -0.0014, -0.0013, -0.0013, -0.0013, -0.0012, -0.0012, -0.0011, -0.0012, -0.0011, -0.001, -0.001, -0.0011, -0.001];
% position_unmodified = pos1_unmodified-pos2_unmodified;
% force2 = ones(1,32)*1.96;%convert to Newton
% stiff_unmodified(:)=(force2(:)./position_unmodified(:));
% % 
% hold on
% 
% plot(k(1:22),k(1:22),'r','LineWidth',2)
% g_modified = fit(k(1:22)',stiff_modified(1:22)','poly2','Normalize','on','Robust','Bisquare');
% % a=plot(g_modified,k(1:22)',stiff_modified(1:22)','.')
% % set(a,'color','k','LineWidth',2)
% g_unmodified = fit(k(1:32)',stiff_unmodified(1:32)','poly2','Normalize','on','Robust','Bisquare');
% g_prime = fit(stiff_unmodified(1:32)',k(1:32)','poly3','Normalize','on','Robust','Bisquare');
% b=plot(g_unmodified,k(1:32)',stiff_unmodified(1:32)','.')
% b=plot(g_prime,stiff_unmodified(1:32)',k(1:32)','.')
% set(b,'color','b','LineWidth',2)
%  legend('Input','Modified Output','Modified Fit','Unmodified Output','Unmodified Fit')
% ylabel('Actual Stiffness')
% xlabel('Desired Stiffness')
% % 
% for i=1:32
%     g(i)=g_prime(i*100)
% end
% 


% pos1_modified =[-0.019,-0.009,-0.005,-0.005,-0.0045,-0.004,-0.004,-0.003,-0.003,-0.002,-0.002,-0.002,-0.002,-0.00195,-0.00176,-0.00165,-0.00145,-0.00113,-0.0015,-0.00137,-0.0012,-0.0012,-0.00128,-0.0012,-0.00126,-0.00146,-0.0012,-0.0014,-0.00126];
% pos2_modified=[-0.032,-0.017,-0.013,-0.010,-0.009,-0.008,-0.007,-0.0061,-0.0058,-0.0057,-0.0057,-0.0057,-0.005,-0.005,-0.005,-0.0047,-0.0045,-0.0045,-0.0045,-0.0045,-0.0043,-0.0044,-0.0044,-0.004,-0.0039,-0.0038,-0.0036,-0.0037,-0.004];
% 
%  pos1_unmodified=[-0.006,-0.0044,-0.0032,-0.00197,-0.00099,-0.001,-0.002,-0.0005,-0.0012,-0.0009,-0.00139,-0.0007,-0.00116,-0.00097,-0.001,-0.00079,-0.0007,-0.00039,-0.00064,-0.00086,-0.00068,-0.0007,-0.001,-0.0012,-0.001,-0.0009,-0.0007,-0.000997,-0.00084];
%  pos2_unmodified=[-0.035,-0.0265,-0.0206,-0.0148,-0.0137,-0.012,-0.01,-0.009,-0.009,-0.0082,-0.007,-0.006,-0.006,-0.0052,-0.0056,-0.0052,-0.0048,-0.0045,-0.0043,-0.0038,-0.00388,-0.00355,-0.0031,-0.003,-0.0027,-0.0027,-0.0026,-0.0025,-0.0023];
%  position_unmodified = pos1_unmodified-pos2_unmodified;
% force = ones(1,29)*4.12;%convert to Newton
% stiff_unmodified(:)=(force(:)./position_unmodified(:));
% for i=2:30
%    k(1,i)=i*100;
% end
% k=k(2:30);
% hold on
%plot(k,k)
% g_unmodified = fit(k(1:29)',stiff_unmodified(1:29)','poly2','Normalize','on','Robust','Bisquare');
% a=plot(g_unmodified,k(1:29)',stiff_unmodified(1:29)','.')
%legend('Input','Modified Output','Modified Fit','Unmodified Output','Unmodified Fit')
ylabel('Actual Stiffness')
xlabel('Desired Stiffness')
%g_prime = fit(stiff_unmodified(1:29)',k(1:29)','poly3','Normalize','on','Robust','Bisquare');
%figure
%b=plot(g_prime,stiff_unmodified(1:29)',k(1:29)','.')
%for i=1:29
%    g(i)=g_prime(100*i)
%end
% % % hold on
% % %  position_modified = pos1_modified-pos2_modified;
% % % force = ones(1,29)*4.12;%convert to Newton
% % % stiff_modified(:)=(force(:)./position_modified(:));
% % % g_modified = fit(k(1:29)',stiff_modified(1:29)','poly2','Normalize','on','Robust','Bisquare');
% % % plot(g_modified,k(1:29)',stiff_modified(1:29)','.')

 pos1_unmodified=[-0.018,-0.009,-0.007,-0.0059,-0.00457,-0.0034,-0.0032,-0.0029,-0.00269,-0.0028,-0.0027,-0.0024,-0.002267,-0.002,-0.00187,-0.00181,-0.001799,-0.00164,-0.00166,-0.00163,-0.001628,-0.0016,-0.00139,-0.0015,-0.00142,-0.00123,-0.00128,-0.0012,-0.0012];
 pos2_unmodified=[-0.028,-0.025,-0.019,-0.0156,-0.0130,-0.0112,-0.00997,-0.0087,-0.0075,-0.0068,-0.0063,-0.0061,-0.0058,-0.00568,-0.0051,-0.0048,-0.00477,-0.0043,-0.004,-0.0034,-0.0032,-0.003,-0.0031,-0.00316,-0.00286,-0.00263,-0.00262,-0.0026,-0.0027];
 position_unmodified = pos1_unmodified-pos2_unmodified;
force = ones(1,29)*3.21;%convert to Newton
stiff_unmodified(:)=(force(:)./position_unmodified(:));
for i=2:30
   k(1,i)=i*100;
end
k=k(2:30);
hold on
plot(k,k,'k','LineWidth',2)
g_unmodified = fit(k(1:29)',stiff_unmodified(1:29)','poly2','Normalize','on','Robust','Bisquare');
b=plot(g_unmodified,k(1:29)',stiff_unmodified(1:29)','.');
set(b,'color','b','LineWidth',2)
g_prime = fit(stiff_unmodified(1:29)',k(1:29)','poly2','Normalize','on','Robust','Bisquare');
%x2=1:3000; y2=polyval(g_prime,x2);plot(stiff_unmodified(1:29)',k(1:29)','o',x2,y2)
% figure
%b=plot(g_prime,stiff_unmodified(1:29)',k(1:29)','.')
for i=1:30
   g(i)=g_prime(100*i)
end
pos1_modified=[-0.012,-0.0048,-0.003,-0.003,-0.0025,-0.0024,-0.0022,-0.00187,-0.00164,-0.00143,-0.00142,-0.00163,-0.001696,-0.00162,-0.00158,-0.0015,-0.001,-0.00115,-0.00119,-0.0012,-0.001,-0.001,-0.001,-0.001,-0.0012,-0.001,-0.0009,-0.001,-0.001];
pos2_modified=[-0.024,-0.0163,-0.0114,-0.0095,-0.00718,-0.0064,-0.006,-0.0052,-0.0047,-0.0042,-0.0039,-0.0038,-0.0038,-0.0035,-0.0032,-0.0032,-0.0029,-0.00285,-0.0027,-00027,-0.0027,-0.0026,-0.002553,-0.002427,-0.00236,-0.0023,-0.00214,-0.00205,-0.00206];
hold on
 position_modified = pos1_modified-pos2_modified;
force = ones(1,29)*3.21;%convert to Newton
stiff_modified(:)=(force(:)./position_modified(:));
g_modified = fit(k(1:29)',stiff_modified(1:29)','poly2','Normalize','on','Robust','Bisquare');
a=plot(g_modified,k(1:29)',stiff_modified(1:29)','.')
set(a,'color','r','LineWidth',2)
legend('Input','Unmodified Output','Poly2 Fit- Unmodified Output','Modified Output','Poly2 Fit- Modified Output')
ylabel('Actual Stiffness')
xlabel('Desired Stiffness')